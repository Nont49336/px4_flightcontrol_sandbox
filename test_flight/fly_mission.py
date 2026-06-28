#!/usr/bin/env python3
"""
fly_mission.py — minimal autonomous flight against PX4 SITL using pymavlink.

Mission profile
---------------
    1. Connect and wait for (a) a position estimate and (b) "ready to arm".
    2. Switch to OFFBOARD and arm.
    3. Climb straight up to 50 m above the takeoff point.
    4. Fly 10 m "forward" (North, holding a North-facing heading).
    5. Hand control to PX4's AUTO.LAND mode and wait for touchdown + disarm.

Why OFFBOARD + position setpoints?
----------------------------------
PX4 accepts external "go here" commands only in OFFBOARD mode, and only while a
setpoint stream arrives faster than 2 Hz. If the stream stalls for >0.5 s, PX4
triggers a failsafe and drops out of OFFBOARD. This script therefore re-sends the
current setpoint on every loop iteration (~20 Hz), even while it is just waiting
to arrive at a target.

"Forward" == North here: we hold yaw = 0 (nose North) the whole time, so
commanding +North in the local frame is the same as commanding "straight ahead".

Robustness notes (added after debugging a stuck SITL)
-----------------------------------------------------
* We stream a GCS heartbeat at ~1 Hz for the *entire* mission. PX4's preflight
  checks include "connection to the GCS": without a live heartbeat the vehicle
  stays in arming-state INIT (reports MAV_STATE_UNINIT) and refuses to arm. A
  one-time burst at connect is NOT enough — the link goes stale within seconds,
  which is exactly what caused the earlier "failed to arm in OFFBOARD".
* We wait for the vehicle to report MAV_STATE_STANDBY ("ready to arm") before
  arming, and print the real COMMAND_ACK reason if it refuses. If the vehicle
  never becomes ready, you get a clear diagnostic instead of silent retries.

Usage
-----
    # 1) In another terminal, start SITL:    make px4_sitl gz_x500
    # 2) Then run this script:
    python3 fly_mission.py
    python3 fly_mission.py --connect udpin:0.0.0.0:14540 --alt 50 --forward 10
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil

# --- PX4 custom flight modes (main_mode, sub_mode) for MAV_CMD_DO_SET_MODE ----
# PX4 reads param2 as the "main" mode and param3 as the "sub" mode directly.
PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_SUB_MODE_AUTO_LAND = 6

# Human-readable lookups for the codes PX4 sends back.
MAV_RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
              3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}
MAV_STATE = {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY",
             4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF"}
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4

# SET_POSITION_TARGET_LOCAL_NED type_mask: ignore velocity, acceleration and
# yaw-rate fields so PX4 uses only the position (x, y, z) and yaw we provide.
IGNORE_VEL_ACC_YAWRATE = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

STREAM_HZ = 20.0           # setpoint stream rate (must stay > 2 Hz for OFFBOARD)
ARRIVE_RADIUS_M = 0.7      # consider a 3D setpoint "reached" within this distance
SETTLE_TIME_S = 2.0        # ...and hold it for this long before moving on


class Drone:
    """Thin pymavlink wrapper that tracks the latest local position + arm state."""

    def __init__(self, connection_string):
        print(f"[connect] opening {connection_string} ...")
        self.master = mavutil.mavlink_connection(
            connection_string, source_system=255, source_component=190)

        # Emit a few heartbeats so PX4's offboard link learns our address and
        # starts streaming telemetry back to us.
        for _ in range(5):
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(0.1)

        if self.master.wait_heartbeat(timeout=10) is None:
            sys.exit("[fatal] no heartbeat in 10 s — is SITL running on this port?")
        self.sysid = self.master.target_system
        self.compid = 1  # talk to the autopilot component explicitly
        print(f"[connect] heartbeat from system {self.sysid}")

        # State caches, updated by pump().
        self.north = self.east = self.down = None
        self.armed = False
        self.system_status = None
        self.acks = {}  # command id -> last result
        self._last_hb = 0.0  # timestamp of last GCS heartbeat we sent

        # Stream local position at 20 Hz so arrival checks are crisp.
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, STREAM_HZ)

    # --- low-level helpers ----------------------------------------------------
    def _request_message(self, message_id, hz):
        self.master.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )

    def _maybe_heartbeat(self):
        """Emit a GCS heartbeat at most ~1 Hz to keep PX4's GCS link alive.

        This is the difference between arming and a stuck "No connection to the
        GCS" preflight failure. Called from pump(), which runs on every loop.
        """
        now = time.time()
        if now - self._last_hb >= 1.0:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._last_hb = now

    def pump(self):
        """Drain inbound messages, updating cached state. Call this every loop."""
        self._maybe_heartbeat()
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "LOCAL_POSITION_NED":
                self.north, self.east, self.down = msg.x, msg.y, msg.z
            elif mtype == "HEARTBEAT":
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.system_status = msg.system_status
            elif mtype == "COMMAND_ACK":
                self.acks[msg.command] = msg.result
            elif mtype == "STATUSTEXT":
                print(f"        [PX4] {msg.text}")

    def send_setpoint(self, north, east, down, yaw=0.0):
        """Stream one NED position setpoint (this is what keeps OFFBOARD alive)."""
        self.master.mav.set_position_target_local_ned_send(
            0,                                       # time_boot_ms (0 = ignore)
            self.sysid, self.compid,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            IGNORE_VEL_ACC_YAWRATE,
            north, east, down,                       # position (m, NED)
            0.0, 0.0, 0.0,                           # velocity (ignored)
            0.0, 0.0, 0.0,                           # acceleration (ignored)
            yaw, 0.0,                                # yaw (rad), yaw_rate (ignored)
        )

    def set_mode(self, main_mode, sub_mode=0):
        self.master.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            main_mode, sub_mode, 0, 0, 0, 0,
        )

    def arm(self):
        self.acks.pop(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, None)
        self.master.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,                     # param1 = 1 -> arm
        )

    # --- higher-level building blocks ----------------------------------------
    def wait_for_position(self, timeout=30.0):
        print("[init] waiting for a valid local position estimate (EKF2) ...")
        start = time.time()
        while time.time() - start < timeout:
            self.pump()
            if self.north is not None:
                print(f"[init] position acquired: "
                      f"N={self.north:.2f} E={self.east:.2f} D={self.down:.2f}")
                return True
            time.sleep(0.1)
        return False

    def wait_until_ready(self, timeout=40.0):
        """Wait until PX4 reports it is ready to arm (MAV_STATE_STANDBY)."""
        print("[init] waiting for 'ready to arm' (system_status = STANDBY) ...")
        start = time.time()
        last_print = 0.0
        while time.time() - start < timeout:
            self.pump()
            if self.system_status in (MAV_STATE_STANDBY, MAV_STATE_ACTIVE):
                print("[init] vehicle is READY TO ARM")
                return True
            if time.time() - last_print > 3.0:
                state = MAV_STATE.get(self.system_status, self.system_status)
                print(f"        ...still not ready (system_status = {state})")
                last_print = time.time()
            time.sleep(0.1)
        state = MAV_STATE.get(self.system_status, self.system_status)
        print(f"[fatal] vehicle never became ready (stuck at system_status = {state}).")
        print("        This is a vehicle/SITL state problem, not a script problem.")
        print("        In the PX4 (pxh>) shell run:  commander check")
        print("        A wedged SITL usually clears with a clean restart.")
        return False

    def goto(self, north, east, down, label, timeout=90.0):
        """Stream a setpoint until the vehicle arrives and settles, or times out."""
        print(f"[goto] {label}: target N={north:.1f} E={east:.1f} D={down:.1f} "
              f"(alt {-down:.1f} m)")
        period = 1.0 / STREAM_HZ
        start = time.time()
        settled_since = None
        while time.time() - start < timeout:
            self.send_setpoint(north, east, down)
            self.pump()
            if self.north is not None:
                dist = math.sqrt((north - self.north) ** 2
                                 + (east - self.east) ** 2
                                 + (down - self.down) ** 2)
                if dist < ARRIVE_RADIUS_M:
                    settled_since = settled_since or time.time()
                    if time.time() - settled_since >= SETTLE_TIME_S:
                        print(f"[goto] {label}: reached (residual {dist:.2f} m)")
                        return True
                else:
                    settled_since = None
            time.sleep(period)
        print(f"[goto] {label}: TIMEOUT after {timeout:.0f} s")
        return False

    def stream_for(self, north, east, down, seconds):
        """Hold a setpoint for a fixed time (used to prime the OFFBOARD stream)."""
        period = 1.0 / STREAM_HZ
        end = time.time() + seconds
        while time.time() < end:
            self.send_setpoint(north, east, down)
            self.pump()
            time.sleep(period)


def main():
    parser = argparse.ArgumentParser(description="PX4 SITL up/forward/land demo.")
    parser.add_argument("--connect", default="udpin:0.0.0.0:14540",
                        help="MAVLink connection string (default: udpin:0.0.0.0:14540)")
    parser.add_argument("--alt", type=float, default=50.0,
                        help="climb altitude in metres (default: 50)")
    parser.add_argument("--forward", type=float, default=10.0,
                        help="forward (North) distance in metres (default: 10)")
    args = parser.parse_args()

    drone = Drone(args.connect)

    if not drone.wait_for_position():
        sys.exit("[fatal] no position estimate — is SITL running and GPS locked?")

    # Gate on "ready to arm". This is what turns a mysterious arm failure into a
    # clear message about *why* the vehicle won't arm.
    if not drone.wait_until_ready():
        sys.exit(1)

    # Capture the takeoff point so the mission is relative to "here".
    home_n, home_e = drone.north, drone.east
    cruise_down = -args.alt                       # NED: up is negative

    # 1) Prime the OFFBOARD stream: PX4 needs setpoints flowing *before* it will
    #    accept the mode switch. Hold the current spot for ~1.5 s first.
    print("[mode] priming setpoint stream for OFFBOARD ...")
    drone.stream_for(home_n, home_e, drone.down, seconds=1.5)

    # 2) Switch to OFFBOARD, then arm. Report the real ACK reason on failure.
    arm_cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    for attempt in range(1, 6):
        drone.set_mode(PX4_MAIN_MODE_OFFBOARD)
        drone.arm()
        drone.stream_for(home_n, home_e, drone.down, seconds=1.0)
        if drone.armed:
            break
        reason = MAV_RESULT.get(drone.acks.get(arm_cmd), "no ACK")
        print(f"[mode] not armed (attempt {attempt}/5, arm result: {reason}), retrying ...")
    if not drone.armed:
        sys.exit("[fatal] failed to arm in OFFBOARD")
    print("[mode] OFFBOARD + ARMED")

    # 3) Climb to target altitude, straight up.
    if not drone.goto(home_n, home_e, cruise_down, label="climb to 50 m"):
        print("[warn] climb did not fully settle — continuing anyway")

    # 4) Fly forward (North) by the requested distance, holding altitude.
    drone.goto(home_n + args.forward, home_e, cruise_down, label="forward 10 m")

    # 5) Land where we are. AUTO.LAND hands the descent + disarm to PX4 itself.
    print("[land] switching to AUTO.LAND ...")
    for _ in range(5):
        drone.set_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_LAND)
        time.sleep(0.2)
        drone.pump()

    # Wait for PX4 to descend and auto-disarm on ground contact.
    print("[land] descending — waiting for auto-disarm ...")
    start = time.time()
    while time.time() - start < 90.0:
        drone.pump()
        if not drone.armed:
            print("[land] touchdown confirmed, disarmed. Mission complete.")
            return
        time.sleep(0.2)
    print("[land] timed out waiting for disarm (vehicle may still be settling)")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[abort] interrupted by user")
