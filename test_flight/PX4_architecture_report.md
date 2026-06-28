# How PX4 Works — A Practical Architecture Report

A guided tour of the PX4 flight stack, written to be read alongside
[`PX4_flow_diagram.md`](./PX4_flow_diagram.md) and the demo script
[`fly_mission.py`](./fly_mission.py). The goal: by the end you should be able to
point at any part of a flight and say *which module did that, and which message
carried the decision*.

---

## 1. The one big idea: modules + a message bus (uORB)

PX4 is **not** one giant control loop. It is a few dozen small, independent
programs ("modules") that never call each other directly. Instead they
communicate through **uORB**, a publish/subscribe message bus.

- A module **publishes** typed messages to a named **topic**
  (e.g. `vehicle_attitude`).
- Other modules **subscribe** to topics they care about.
- Topics are defined as `.msg` files in `msg/` and compiled into C++ structs.

Why this matters:

- **Decoupling** — the position controller doesn't know or care *who* produced
  the position estimate. Swap EKF2 for another estimator and nothing downstream
  changes, as long as it publishes `vehicle_local_position`.
- **Observability** — you can inspect any topic live (`listener vehicle_attitude`
  in the pxh shell), which makes the whole system debuggable.
- **Testability** — modules can be run and tested in isolation.

> If you remember one thing: **arrows in the diagram are uORB topics.** Following
> a flight = following topics from sensors to motors.

---

## 2. Where the code runs: NuttX, POSIX, and SITL

The same flight-control source compiles for very different targets:

| Target | What it is | How modules run |
|--------|-----------|-----------------|
| **NuttX** (e.g. Pixhawk) | Real-time OS on the flight controller MCU | Real tasks / work queues on hardware |
| **POSIX / SITL** | The firmware as a normal Linux/macOS process | Same modules, scheduled in threads |
| **SITL + simulator** | SITL wired to Gazebo/jMAVSim/Gazebo-classic | Physics replaces real sensors & motors |

`make px4_sitl gz_x500` builds the **POSIX** target and launches it against the
**Gazebo** `x500` quadcopter. The flight code is *identical* to what flies on
hardware — only the bottom layer (sensors in, motor PWM out) is swapped for a
simulator. That's what makes SITL trustworthy for learning and testing.

### Lockstep simulation
PX4 SITL runs in **lockstep** with the simulator: simulation time is the master
clock, and PX4 and Gazebo step together. This makes runs deterministic and lets a
slow machine still simulate "real time" correctly — but it also means **if the
simulator stalls, PX4's clock stalls too.** (This is also why the protobuf/Gazebo
toolchain has to be consistent for the build to even produce `bin/px4`.)

---

## 3. How a module is structured

Most modules are one of two shapes:

- **Tasks** — run their own loop (e.g. `commander`, `navigator`, `mavlink`).
- **Work-queue items** — woken by a topic update or a timer, do a slice of work,
  yield (e.g. the controllers, `ekf2`). This is cheaper and more cache-friendly
  on an MCU.

A controller's life is basically:

```
on new input topic (e.g. vehicle_attitude_setpoint):
    read latest estimate (vehicle_attitude)
    compute control law
    publish output topic (vehicle_rates_setpoint)
```

You can see them all and their CPU/stack usage at runtime by typing `top` or
`work_queue status` in the PX4 shell.

---

## 4. The full pipeline, node by node

This is the narrative version of diagram #1. Follow it top to bottom.

### 4.1 Sensing — `sensors`, `rc_update`
Raw drivers produce noisy, redundant data (often *several* IMUs). The **`sensors`**
module validates instances, applies calibration and board rotation, and republishes
clean signals: `vehicle_imu`, `vehicle_magnetometer`, `vehicle_air_data`.
**`rc_update`** decodes pilot stick channels into `manual_control_setpoint`.

> Contribution to control: this is the *truth intake*. Every downstream decision
> is only as good as what leaves this stage.

### 4.2 State estimation — `ekf2`
The **Extended Kalman Filter** fuses IMU + GPS + magnetometer + barometer (+ flow,
vision, etc.) into one self-consistent estimate of the vehicle's state:

- `vehicle_attitude` — orientation (quaternion)
- `vehicle_local_position` — position/velocity in a local NED frame
- `vehicle_global_position` — lat/lon/alt

> Contribution to control: this is the drone's **sense of self**. The cascade
> below is a feedback system; `ekf2` *is* the feedback. A bad estimate looks
> exactly like a control failure.

### 4.3 Decision & supervision — `commander`, `navigator`, `flight_mode_manager`
- **`commander`** is the safety brain and state machine: pre-arm checks, arming,
  which flight mode is active, and **failsafes** (low battery, GPS loss, RC loss →
  e.g. forced RTL or LAND). It publishes `vehicle_status`,
  `vehicle_control_mode`, and `actuator_armed`.
- **`navigator`** owns *autonomous* geographic intent: missions, RTL, geofence,
  and the **AUTO.TAKEOFF / AUTO.LAND** logic. It emits a
  `position_setpoint_triplet` (previous/current/next waypoint).
- **`flight_mode_manager`** runs **Flight Tasks**. It takes whatever the active
  mode wants — a stick input, a mission waypoint, or an **offboard target from
  your script** — and produces a single **smooth, feasible** `trajectory_setpoint`
  (with jerk/acceleration limits) for the controllers.

> Contribution to control: this layer decides *what the goal is* and *whether it's
> allowed*, then shapes it into something physically trackable.

### 4.4 The cascaded controller — `mc_pos_control` → `mc_att_control` → `mc_rate_control`
This is the heart of flight. Each stage converts a higher-level goal into a
lower-level one (see diagram #2 for rates and feedback):

1. **`mc_pos_control`** (outer, slow): position/velocity error →
   **desired tilt + collective thrust** → `vehicle_attitude_setpoint`.
   *"I'm too low and too far back → lean forward a bit and add thrust."*
2. **`mc_att_control`** (middle): attitude error (where I point vs. where I should)
   → **desired body rotation rates** → `vehicle_rates_setpoint`.
3. **`mc_rate_control`** (inner, fastest): body-rate error → **torque + thrust**
   demand (`vehicle_torque_setpoint`, `vehicle_thrust_setpoint`), using PID +
   feed-forward. This is the loop that actually keeps the craft stable.

> Contribution to control: this is *the* control law. Outer loops are gentle and
> think in metres; the inner loop is stiff and fast and thinks in motor torque.

### 4.5 Allocation & output — `control_allocator`, output drivers
The controllers ask for an abstract **net torque + thrust**. They do **not** know
how many motors exist or where they are. **`control_allocator`** does: it inverts
the airframe's effectiveness matrix (geometry of the 4 rotors on the x500) to turn
that demand into per-motor commands → `actuator_motors`. Output drivers translate
that to PWM/DShot on hardware, or to motor speeds in Gazebo.

> Contribution to control: this is the **bridge from physics-abstract demand to
> the specific machine.** Change the airframe (hexacopter, VTOL), and only this
> mapping changes — the controllers above are untouched.

### 4.6 Closing the loop
Motors → thrust → the airframe accelerates → sensors measure the new motion →
`ekf2` updates the estimate → the controllers see the error shrink. In SITL,
Gazebo plays the role of "physics + sensors + motors". **That's the feedback
loop**, spinning hundreds of times a second.

---

## 5. Flight modes (and where your script fits)
A "flight mode" is essentially *which source feeds `flight_mode_manager`*:

- **Manual / Stabilized / Altitude / Position** — pilot sticks, with increasing
  amounts of autopilot help.
- **AUTO.MISSION / RTL / TAKEOFF / LAND** — `navigator` drives.
- **OFFBOARD** — an external computer streams setpoints over MAVLink. **This is
  what `fly_mission.py` uses.**

PX4 enforces two rules for OFFBOARD that the script is built around:
1. Setpoints must stream **> 2 Hz** *before* and *during* the mode, or PX4
   failsafes out. (The script streams at 20 Hz, even while waiting.)
2. Mode + arming are gated by `commander`'s health checks (needs a valid
   position estimate). The script waits for `ekf2` before arming.

---

## 6. Safety: failsafes & the land detector
`commander` continuously evaluates failsafe conditions (battery, RC/GPS/datalink
loss, geofence). `land_detector` publishes `vehicle_land_detected` from thrust +
motion cues so that **AUTO.LAND knows when the wheels are down** and can
auto-disarm. The demo relies on exactly this: after switching to AUTO.LAND, the
script just waits for the disarm that `land_detector` → `commander` produce.

---

## 7. Mapping the demo flight onto the architecture

| Mission step (`fly_mission.py`) | What PX4 does internally |
|---------------------------------|--------------------------|
| Connect + wait for position | `mavlink` bridges UDP; script waits for `ekf2`'s `vehicle_local_position` |
| Stream setpoints, set OFFBOARD, arm | `commander` checks health, enables offboard `vehicle_control_mode`, arms |
| Climb to 50 m | `trajectory_setpoint` (D=-50) → pos→att→rate cascade → `control_allocator` → motors |
| Forward 10 m (North) | same cascade, new `trajectory_setpoint` (N=+10), heading held at 0 |
| AUTO.LAND | `navigator` runs the descent; `land_detector` triggers auto-disarm |

Every one of those internal steps is a topic you can watch live in the PX4 shell
while the script runs (e.g. `listener trajectory_setpoint`,
`listener vehicle_attitude_setpoint`, `listener actuator_motors`).

---

## 8. Where to look in the source

| Area | Path |
|------|------|
| uORB topic definitions | `msg/` |
| Sensor aggregation | `src/modules/sensors/` |
| State estimator | `src/modules/ekf2/` |
| Safety / arming / modes | `src/modules/commander/` |
| Missions / RTL / land | `src/modules/navigator/` |
| Flight Tasks (setpoint shaping) | `src/modules/flight_mode_manager/` |
| Position controller | `src/modules/mc_pos_control/` |
| Attitude controller | `src/modules/mc_att_control/` |
| Rate controller | `src/modules/mc_rate_control/` |
| Control allocation / mixing | `src/modules/control_allocator/` |
| Land detection | `src/modules/land_detector/` |
| MAVLink bridge (offboard, telemetry) | `src/modules/mavlink/` |
| SITL startup scripts | `ROMFS/px4fmu_common/init.d-posix/` |
| Simulator interface | `src/modules/simulation/` |

---

## 9. TL;DR

PX4 is a **publish/subscribe pipeline**: sensors → `ekf2` estimate → a
decision layer (`commander`/`navigator`/`flight_mode_manager`) that produces a
**trajectory setpoint** → a **cascaded controller** (position → attitude → rate)
→ `control_allocator` → motors → physics → back to sensors. The same code runs in
SITL and on real hardware; only the bottom layer changes. Your `fly_mission.py`
plugs into one entry point of that pipeline (a `trajectory_setpoint` via OFFBOARD)
and lets PX4's normal machinery do the actual flying.
