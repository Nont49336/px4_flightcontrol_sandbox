# test_flight

A minimal end-to-end demo of autonomous flight on PX4 SITL, plus documentation
that explains how PX4 actually controls the drone.

## Contents

| File | What it is |
|------|------------|
| [`fly_mission.py`](./fly_mission.py) | Flight script: climb to **50 m**, fly **10 m forward**, then **land**. |
| [`PX4_architecture_report.md`](./PX4_architecture_report.md) | Readable report on how the PX4 flight stack works. |
| [`PX4_flow_diagram.md`](./PX4_flow_diagram.md) | Detailed Mermaid flow diagrams: every node + the uORB topic on each arrow. |

## Run it

**Terminal 1 — start the simulator** (from the repo root):

```bash
make px4_sitl gz_x500
```

Wait until you see the `pxh>` prompt and the EKF has a position fix.

**Terminal 2 — run the mission** (uses the repo's venv, which has `pymavlink`):

```bash
.venv/bin/python test_flight/fly_mission.py
```

Optional flags:

```bash
.venv/bin/python test_flight/fly_mission.py \
    --connect udpin:0.0.0.0:14540 \
    --alt 50 \
    --forward 10
```

## What you'll see

The script prints each phase: connect → position acquired → OFFBOARD + ARMED →
climb to 50 m → forward 10 m → AUTO.LAND → disarm. In the PX4 shell you can watch
the very same data flowing through the stack:

```text
pxh> listener trajectory_setpoint        # the goal the script injects
pxh> listener vehicle_attitude_setpoint  # position controller's output
pxh> listener actuator_motors            # final per-motor commands
```

## How it works (one paragraph)

The script talks **only** to PX4's `mavlink` module over UDP. It streams a
position target (this is OFFBOARD mode, which requires setpoints faster than
2 Hz) and flips flight modes. Everything else — turning "go to 50 m" into motor
commands — is PX4's normal cascaded controller:
`trajectory_setpoint → mc_pos_control → mc_att_control → mc_rate_control →
control_allocator → motors`. See the report and diagram for the full story.

> **Note — "forward" = North:** the script holds a North-facing heading (yaw 0)
> and commands +North, so "forward 10 m" is unambiguous. Change `--forward` or the
> yaw in `fly_mission.py` if you want a different direction.
