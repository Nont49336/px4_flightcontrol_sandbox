# PX4 Data-Flow Diagram — How the Drone Is Actually Controlled

This file is the **map you can follow**. It shows every major node (PX4 module),
what message (uORB topic) flows between them, and — below the diagrams — what each
node *does* and *why it matters*. Render the Mermaid blocks in VS Code (Markdown
Preview), GitHub, or https://mermaid.live.

> **Reading the arrows:** each arrow is a uORB topic — a named, strongly-typed
> message channel. A module *publishes* to a topic; any number of modules
> *subscribe*. Nobody calls anybody directly; they just swap messages on the bus.
> This is the single most important idea in PX4.

---

## 1. The whole loop, end to end

```mermaid
flowchart TD
    subgraph SIM["🌍 Simulator (SITL only) — Gazebo gz_x500"]
        PHYS["Rigid-body physics<br/>+ rotor aerodynamics"]
        SIMSENS["Simulated sensors<br/>IMU · mag · baro · GPS"]
        MOTORS_SIM["Motor models<br/>(thrust per rotor)"]
    end

    subgraph SENSE["📡 Sensing layer"]
        SENSORS["<b>sensors</b><br/>fuses/validates raw IMU, mag,<br/>baro into clean vehicle signals"]
        RC["<b>rc_update</b><br/>decodes pilot stick input"]
    end

    subgraph EST["🧭 State estimation"]
        EKF2["<b>ekf2</b><br/>Extended Kalman Filter:<br/>fuses all sensors into ONE<br/>consistent state estimate"]
    end

    subgraph DECIDE["🧠 Decision &amp; mode logic"]
        COMMANDER["<b>commander</b><br/>arming, mode switching,<br/>safety / failsafe state machine"]
        NAV["<b>navigator</b><br/>autonomous missions, RTL,<br/>auto-takeoff &amp; AUTO.LAND"]
        FMM["<b>flight_mode_manager</b><br/>(Flight Tasks)<br/>turns pilot/auto intent into a<br/>smooth trajectory setpoint"]
    end

    subgraph CTRL["🎛️ Control cascade (see diagram 2)"]
        POS["<b>mc_pos_control</b>"]
        ATT["<b>mc_att_control</b>"]
        RATE["<b>mc_rate_control</b>"]
    end

    subgraph ACT["⚙️ Allocation &amp; output"]
        ALLOC["<b>control_allocator</b><br/>splits torque+thrust demand<br/>across the 4 motors"]
        OUT["PWM / DShot / sim out<br/>(actuator_outputs)"]
    end

    OFFBOARD["💻 Offboard API<br/>(<b>mavlink</b> module)<br/>← fly_mission.py / MAVSDK"]

    %% ---- sensing path ----
    SIMSENS -->|"sensor_combined,<br/>sensor_gps, ..."| SENSORS
    SENSORS -->|"vehicle_imu,<br/>vehicle_magnetometer,<br/>vehicle_air_data"| EKF2

    %% ---- estimation outputs feed EVERYTHING ----
    EKF2 -->|"vehicle_attitude"| ATT
    EKF2 -->|"vehicle_local_position"| POS
    EKF2 -->|"vehicle_local_position,<br/>vehicle_global_position"| NAV
    EKF2 -->|"vehicle_local_position"| COMMANDER

    %% ---- decision path ----
    RC --> COMMANDER
    OFFBOARD -->|"vehicle_command,<br/>offboard_control_mode"| COMMANDER
    OFFBOARD -->|"trajectory_setpoint<br/>(position target)"| FMM
    COMMANDER -->|"vehicle_status,<br/>vehicle_control_mode,<br/>actuator_armed"| FMM
    COMMANDER -->|"vehicle_control_mode"| POS
    COMMANDER -->|"vehicle_status"| NAV
    NAV -->|"position_setpoint_triplet"| FMM
    FMM -->|"trajectory_setpoint"| POS

    %% ---- control cascade ----
    POS -->|"vehicle_attitude_setpoint"| ATT
    ATT -->|"vehicle_rates_setpoint"| RATE
    RATE -->|"vehicle_torque_setpoint,<br/>vehicle_thrust_setpoint"| ALLOC

    %% ---- actuation ----
    ALLOC -->|"actuator_motors"| OUT
    OUT -->|"PWM / sim"| MOTORS_SIM
    MOTORS_SIM --> PHYS
    PHYS -->|"closes the loop"| SIMSENS

    classDef est fill:#e8f0ff,stroke:#3366cc,stroke-width:2px;
    classDef ctrl fill:#fff0e6,stroke:#cc6600,stroke-width:2px;
    classDef dec fill:#eafbea,stroke:#2e8b57,stroke-width:2px;
    class EKF2 est;
    class POS,ATT,RATE,ALLOC ctrl;
    class COMMANDER,NAV,FMM dec;
```

---

## 2. Zoom-in: the cascaded controller (the heart of flight)

Each stage takes a *higher-level* goal and produces a *lower-level* goal for the
next stage. Outer loops run slower and think in metres; inner loops run fast and
think in motor torque. This is **cascaded control**.

```mermaid
flowchart LR
    SP["trajectory_setpoint<br/><i>where to be</i><br/>(position / velocity)"] --> POS["<b>mc_pos_control</b><br/>P + PID<br/>position → tilt &amp; thrust"]
    POS -->|"vehicle_attitude_setpoint<br/><i>which way to point</i><br/>(quaternion + thrust)"| ATT["<b>mc_att_control</b><br/>P controller<br/>attitude error → body rates"]
    ATT -->|"vehicle_rates_setpoint<br/><i>how fast to rotate</i><br/>(roll/pitch/yaw rate)"| RATE["<b>mc_rate_control</b><br/>PID + feed-forward<br/>rate error → torque"]
    RATE -->|"vehicle_torque_setpoint<br/>vehicle_thrust_setpoint<br/><i>net moment + lift</i>"| ALLOC["<b>control_allocator</b><br/>inverts the mixer:<br/>4 motor commands"]
    ALLOC -->|"actuator_motors<br/>(0..1 per rotor)"| MTR["4 × ESC / rotor"]
    EKF["<b>ekf2 estimate</b>"] -.->|"vehicle_local_position<br/>(feedback)"| POS
    EKF -.->|"vehicle_attitude<br/>(feedback)"| ATT
    EKF -.->|"angular velocity<br/>(feedback)"| RATE
    classDef ctrl fill:#fff0e6,stroke:#cc6600,stroke-width:2px;
    class POS,ATT,RATE,ALLOC ctrl;
```

**Loop rates (typical, multicopter):**

| Stage | Module | Rate | Thinks in | Output |
|-------|--------|------|-----------|--------|
| Position | `mc_pos_control` | ~50 Hz | metres, m/s | tilt + collective thrust |
| Attitude | `mc_att_control` | ~250 Hz | quaternion error | desired body rates |
| Rate | `mc_rate_control` | ~1 kHz (sim: sensor rate) | rad/s | torque + thrust |
| Allocation | `control_allocator` | with rate loop | per-motor 0..1 | `actuator_motors` |

The rule of thumb: **outer loops are slow and forgiving, inner loops are fast and
stiff.** If the inner rate loop is unstable, nothing above it can save the drone.

---

## 3. What each node does and how it *contributes to control*

| Node (module) | Subscribes to (in) | Publishes (out) | Its job / why it matters |
|---------------|--------------------|------------------|--------------------------|
| **sensors** | raw driver topics | `vehicle_imu`, `vehicle_magnetometer`, `vehicle_air_data` | Picks healthy sensor instances, applies calibration/rotation, hands clean signals up. Garbage in here = crash. |
| **ekf2** | `vehicle_imu`, `sensor_gps`, mag, baro | `vehicle_attitude`, `vehicle_local_position`, `vehicle_global_position` | The drone's sense of *where am I and which way am I pointing*. Every controller below trusts this single estimate. |
| **commander** | `vehicle_command`, RC, EKF health | `vehicle_status`, `vehicle_control_mode`, `actuator_armed` | The referee: decides if you may arm, which flight mode is active, and triggers failsafes (e.g. drop to LAND on GPS loss). |
| **navigator** | `vehicle_global_position`, mission storage | `position_setpoint_triplet` | The autonomous planner: missions, RTL, and the AUTO.TAKEOFF / **AUTO.LAND** your script hands off to. |
| **flight_mode_manager** | `trajectory_setpoint` (offboard), `position_setpoint_triplet`, RC | `trajectory_setpoint` (smoothed), `vehicle_local_position_setpoint` | Runs "Flight Tasks". Converts *intent* (a stick, a waypoint, an offboard target) into a **smooth, feasible** trajectory the position loop can track. |
| **mc_pos_control** | `trajectory_setpoint`, `vehicle_local_position` | `vehicle_attitude_setpoint` | Outer loop. "I'm 50 m below target → tilt this much and push this much thrust." Position/velocity error → desired tilt + thrust. |
| **mc_att_control** | `vehicle_attitude_setpoint`, `vehicle_attitude` | `vehicle_rates_setpoint` | Middle loop. Attitude error (quaternion) → how fast to rotate to fix it. |
| **mc_rate_control** | `vehicle_rates_setpoint`, gyro | `vehicle_torque_setpoint`, `vehicle_thrust_setpoint` | Inner loop, fastest. Body-rate error → required torque (PID + feed-forward). The stabilising core. |
| **control_allocator** | `vehicle_torque_setpoint`, `vehicle_thrust_setpoint` | `actuator_motors`, `actuator_servos` | Solves "what motor commands produce this net torque + thrust?" for the specific airframe geometry. |
| **land_detector** | local position, thrust, actuator state | `vehicle_land_detected` | Tells commander/navigator when ground contact happened, so AUTO.LAND can auto-disarm. |
| **mavlink** | UDP from GCS / offboard | bridges uORB ↔ MAVLink | The radio/USB/UDP bridge. This is the module **`fly_mission.py` talks to** to inject setpoints and read telemetry. |

---

## 4. How your `fly_mission.py` rides this pipeline

```mermaid
sequenceDiagram
    participant S as fly_mission.py
    participant M as mavlink (module)
    participant C as commander
    participant F as flight_mode_manager
    participant P as mc_pos_control → ... → motors

    S->>M: SET_POSITION_TARGET_LOCAL_NED (stream @20Hz)
    M->>F: trajectory_setpoint
    S->>M: DO_SET_MODE = OFFBOARD
    M->>C: vehicle_command
    C-->>F: vehicle_control_mode (offboard enabled)
    S->>M: ARM
    M->>C: vehicle_command (arm)
    Note over C: checks EKF health → arms
    loop until target reached (climb, then forward)
        S->>M: position setpoint (N,E,D)
        M->>F: trajectory_setpoint
        F->>P: smoothed setpoint → cascade → motors spin
    end
    S->>M: DO_SET_MODE = AUTO.LAND
    M->>C: vehicle_command
    C->>F: navigator takes over → descend
    Note over C: land_detector fires → auto-disarm
    C-->>S: HEARTBEAT (disarmed) → script exits
```

The key insight: **your script never touches motors.** It only injects a
`trajectory_setpoint` and flips a mode. Everything from "go to N=10, D=-50" down
to "spin motor 3 at 62%" is PX4's cascade doing its job — exactly the same code
path a real pilot's autonomous flight would use.
