#!/usr/bin/env python3
"""
fly_evtol_mission.py — a scripted eVTOL (VTOL) flight against PX4 SITL.

Mission profile (standard_vtol)
-------------------------------
    1. Arm in multicopter (MC) mode and climb straight up to 50 m.
    2. Transition to fixed-wing (FW): the pusher spins up, the wings take the load.
    3. Cruise 100 m North as an airplane.
    4. Transition back to multicopter (MC): decelerate into a hover.
    5. Land vertically (AUTO.LAND) and disarm on touchdown.

Why a VTOL flies differently from a quad
----------------------------------------
A "standard VTOL" is a quadrotor (4 lift motors) bolted onto an airplane (1 pusher
motor + ailerons/elevator/rudder). PX4 runs *both* a multicopter and a fixed-wing
controller and blends them in the `vtol_att_control` module:

    MC mode .......... 4 lift motors hold attitude + position like a quad; pusher off.
    TRANSITION (fwd) . pusher accelerates the craft; as airspeed builds, the wings
                       make lift and the lift motors throttle down.
    FW mode .......... flies like a plane: pusher for thrust, control surfaces for
                       attitude; lift motors off.
    TRANSITION (back). lift motors spin back up; the craft decelerates into a hover.

We drive the transitions explicitly with MAV_CMD_DO_VTOL_TRANSITION
(param1 = 4 -> FW, param1 = 3 -> MC — exactly how PX4's vtol_att_control reads it)
and watch EXTENDED_SYS_STATE.vtol_state to know when each transition completes.

Everything else (OFFBOARD streaming, the continuous ~1 Hz GCS heartbeat that keeps
PX4's arming checks happy, NED position setpoints) works just like the multicopter
demo in ../test_flight/fly_mission.py.

Why OFFBOARD (not AUTO.TAKEOFF) for the climb
---------------------------------------------
PX4 *rejects* a forward (MC->FW) transition while in AUTO_TAKEOFF/LAND/RTL/ORBIT
(see vtol_att_control_main.cpp). OFFBOARD is allowed, so we climb in OFFBOARD and
can transition the instant we are at altitude.

Usage
-----
    # 1) Start SITL in another terminal:   make px4_sitl gz_standard_vtol
    # 2) Then run this script:
    python3 fly_evtol_mission.py
    python3 fly_evtol_mission.py --alt 50 --forward 100
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil

# --- PX4 custom flight modes (main_mode, sub_mode) for MAV_CMD_DO_SET_MODE ----
PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_SUB_MODE_AUTO_LAND = 6

# --- VTOL transition targets (MAV_CMD_DO_VTOL_TRANSITION param1) --------------
# These integers are read straight out of param1 by PX4's vtol_att_control and
# compared against VEHICLE_VTOL_STATE_*; they match the MAVLink MAV_VTOL_STATE enum.
VTOL_STATE_MC = 3          # multicopter
VTOL_STATE_FW = 4          # fixed-wing
MAV_VTOL_STATE = {0: "UNDEFINED", 1: "->FW (transitioning)",
                  2: "->MC (transitioning)", 3: "MC", 4: "FW"}

# Human-readable lookups for the codes PX4 sends back.
MAV_RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
              3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}
MAV_STATE = {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY",
             4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF"}
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4
LANDED_STATE = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR",
                3: "TAKEOFF", 4: "LANDING"}

# SET_POSITION_TARGET_LOCAL_NED type_mask: use only position (x, y, z) + yaw.
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
ARRIVE_RADIUS_M = 1.5      # "reached" tolerance for the MC climb
CRUISE_REACH_M = 5.0       # how close (in North) is "we've gone far enough" in FW
SETTLE_TIME_S = 2.0        # hold the climb target this long before transitioning


class EVtol:
    """pymavlink wrapper that tracks local position, arm state, and VTOL state."""

    def __init__(self, connection_string):
        print(f"[connect] opening {connection_string} ...")
        self.master = mavutil.mavlink_connection(
            connection_string, source_system=255, source_component=190)

        # A few heartbeats so PX4's offboard link learns our address.
        for _ in range(5):
            self._send_gcs_heartbeat()
            time.sleep(0.1)

        if self.master.wait_heartbeat(timeout=10) is None:
            sys.exit("[fatal] no heartbeat in 10 s — is SITL running on this port?")
        self.sysid = self.master.target_system
        self.compid = 1  # talk to the autopilot component
        print(f"[connect] heartbeat from system {self.sysid}")

        # State caches, updated by pump().
        self.north = self.east = self.down = None
        self.armed = False
        self.system_status = None
        self.vtol_state = None
        self.landed_state = None
        self.acks = {}            # command id -> last result
        self._last_hb = 0.0       # timestamp of last GCS heartbeat we sent

        # Stream the messages our state machine depends on.
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, STREAM_HZ)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5.0)

    # --- low-level helpers ---------------------------------------------------
    def _send_gcs_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _maybe_heartbeat(self):
        """Emit a GCS heartbeat at most ~1 Hz so PX4's 'connection to GCS' arming
        check stays satisfied for the whole mission (a one-time burst is not
        enough — the link goes stale within seconds)."""
        now = time.time()
        if now - self._last_hb >= 1.0:
            self._send_gcs_heartbeat()
            self._last_hb = now

    def _request_message(self, message_id, hz):
        self.master.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )

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
            elif mtype == "EXTENDED_SYS_STATE":
                if msg.vtol_state != self.vtol_state:
                    print(f"        [vtol] state -> {MAV_VTOL_STATE.get(msg.vtol_state, msg.vtol_state)}")
                self.vtol_state = msg.vtol_state
                self.landed_state = msg.landed_state
            elif mtype == "COMMAND_ACK":
                self.acks[msg.command] = msg.result
            elif mtype == "STATUSTEXT":
                print(f"        [PX4] {msg.text}")

    def send_setpoint(self, north, east, down, yaw=0.0):
        """Stream one NED position setpoint (this is what keeps OFFBOARD alive)."""
        self.master.mav.set_position_target_local_ned_send(
            0, self.sysid, self.compid,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            IGNORE_VEL_ACC_YAWRATE,
            north, east, down,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            yaw, 0.0,
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
            1, 0, 0, 0, 0, 0, 0,
        )

    def transition(self, target_state):
        """Command a VTOL transition (target_state = VTOL_STATE_FW or _MC)."""
        where = "FIXED-WING" if target_state == VTOL_STATE_FW else "MULTICOPTER"
        print(f"[vtol] commanding transition to {where} (param1={target_state}) ...")
        self.acks.pop(mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, None)
        self.master.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, 0,
            float(target_state),   # param1: target VTOL state
            0, 0, 0, 0, 0, 0,      # param2=0: normal (not immediate) transition
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

    def wait_until_ready(self, timeout=60.0):
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
        return False

    def stream_for(self, north, east, down, seconds, yaw=0.0):
        """Hold a setpoint for a fixed time (used to prime / settle the stream)."""
        period = 1.0 / STREAM_HZ
        end = time.time() + seconds
        while time.time() < end:
            self.send_setpoint(north, east, down, yaw)
            self.pump()
            time.sleep(period)

    def climb_to(self, north, east, down, timeout=60.0):
        """MC hover-climb: stream a setpoint until we arrive and settle."""
        print(f"[climb] target N={north:.1f} E={east:.1f} alt={-down:.1f} m")
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
                        print(f"[climb] reached 50 m (residual {dist:.2f} m)")
                        return True
                else:
                    settled_since = None
            time.sleep(period)
        print("[climb] TIMEOUT — continuing anyway")
        return False

    def wait_vtol_state(self, target_state, setpoint, timeout=40.0):
        """Stream `setpoint` (n,e,d) until EXTENDED_SYS_STATE.vtol_state == target."""
        name = MAV_VTOL_STATE.get(target_state, target_state)
        print(f"[vtol] waiting for vtol_state == {name} ...")
        period = 1.0 / STREAM_HZ
        start = time.time()
        while time.time() - start < timeout:
            self.send_setpoint(*setpoint)
            self.pump()
            if self.vtol_state == target_state:
                print(f"[vtol] now in {name}")
                return True
            time.sleep(period)
        print(f"[vtol] TIMEOUT waiting for {name} (still "
              f"{MAV_VTOL_STATE.get(self.vtol_state, self.vtol_state)})")
        return False

    def cruise_north(self, target_north, east, down, timeout=90.0):
        """Fly forward (North) as a fixed-wing until we pass target_north."""
        print(f"[cruise] flying North to N={target_north:.1f} m (fixed-wing) ...")
        period = 1.0 / STREAM_HZ
        start = time.time()
        last_print = 0.0
        while time.time() - start < timeout:
            self.send_setpoint(target_north, east, down)
            self.pump()
            if self.north is not None and self.north >= target_north - CRUISE_REACH_M:
                print(f"[cruise] reached North target (N={self.north:.1f} m)")
                return True
            if time.time() - last_print > 3.0 and self.north is not None:
                print(f"        ...N={self.north:.1f} m  alt={-self.down:.1f} m  "
                      f"state={MAV_VTOL_STATE.get(self.vtol_state, self.vtol_state)}")
                last_print = time.time()
            time.sleep(period)
        print(f"[cruise] TIMEOUT — got to N={self.north:.1f} m; transitioning back anyway")
        return False


def main():
    parser = argparse.ArgumentParser(description="PX4 SITL eVTOL up / transition / cruise / land demo.")
    parser.add_argument("--connect", default="udpin:0.0.0.0:14540",
                        help="MAVLink connection string (default: udpin:0.0.0.0:14540)")
    parser.add_argument("--alt", type=float, default=50.0,
                        help="climb altitude in metres (default: 50)")
    parser.add_argument("--forward", type=float, default=100.0,
                        help="North cruise distance in metres (default: 100)")
    args = parser.parse_args()

    v = EVtol(args.connect)

    if not v.wait_for_position():
        sys.exit("[fatal] no position estimate — is SITL running and GPS locked?")
    if not v.wait_until_ready():
        sys.exit("[fatal] vehicle never became ready to arm")

    home_n, home_e = v.north, v.east
    cruise_d = -args.alt                     # NED: up is negative
    target_n = home_n + args.forward

    # --- prime OFFBOARD, then arm (MC) ---------------------------------------
    print("[mode] priming setpoint stream for OFFBOARD ...")
    v.stream_for(home_n, home_e, v.down, seconds=2.0)

    arm_cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    for attempt in range(1, 6):
        v.set_mode(PX4_MAIN_MODE_OFFBOARD)
        v.arm()
        v.stream_for(home_n, home_e, v.down, seconds=1.0)
        if v.armed:
            break
        reason = MAV_RESULT.get(v.acks.get(arm_cmd), "no ACK")
        print(f"[mode] not armed (attempt {attempt}/5, arm result: {reason}), retrying ...")
    if not v.armed:
        sys.exit("[fatal] failed to arm in OFFBOARD")
    print("[mode] OFFBOARD + ARMED (multicopter)")

    # --- 1) climb to 50 m as a multicopter -----------------------------------
    v.climb_to(home_n, home_e, cruise_d)

    # --- 2) transition to fixed-wing -----------------------------------------
    # Stream the forward target while transitioning so the FW controller has a
    # heading + altitude to fly once the wings take over.
    v.transition(VTOL_STATE_FW)
    v.wait_vtol_state(VTOL_STATE_FW, setpoint=(target_n, home_e, cruise_d), timeout=40.0)

    # --- 3) cruise 100 m North as a fixed-wing -------------------------------
    v.cruise_north(target_n, home_e, cruise_d, timeout=90.0)

    # --- 4) transition back to multicopter -----------------------------------
    # Hold position where we are now; the craft decelerates into a hover.
    hold_n = v.north if v.north is not None else target_n
    hold_e = v.east if v.east is not None else home_e
    v.transition(VTOL_STATE_MC)
    v.wait_vtol_state(VTOL_STATE_MC, setpoint=(hold_n, hold_e, cruise_d), timeout=40.0)
    v.stream_for(hold_n, hold_e, cruise_d, seconds=3.0)   # stabilise the hover

    # --- 5) land vertically (AUTO.LAND back-handles descent + disarm) ---------
    print("[land] switching to AUTO.LAND ...")
    for _ in range(5):
        v.set_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_LAND)
        time.sleep(0.2)
        v.pump()

    print("[land] descending — waiting for auto-disarm ...")
    start = time.time()
    while time.time() - start < 120.0:
        v.pump()
        if not v.armed:
            print("[land] touchdown confirmed, disarmed. Mission complete.")
            return
        time.sleep(0.2)
    print("[land] timed out waiting for disarm (vehicle may still be settling)")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[abort] interrupted by user")
