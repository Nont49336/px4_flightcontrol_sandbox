# evtol_testflight

A scripted **eVTOL** flight on PX4 SITL: climb as a multicopter → transition to
fixed-wing → cruise North → transition back → land vertically. Built to *see the
VTOL fly* and to *understand how PX4's VTOL stack works*.

## Files
| File | What it is |
|---|---|
| `fly_evtol_mission.py` | The flight script (pymavlink, OFFBOARD + explicit transitions). |
| `EVTOL_flow_diagram.md` | Follow-along flow diagrams: mission steps, VTOL state machine, script↔PX4 sequence. |
| `EVTOL_architecture.md` | Node-by-node: how each PX4 module contributes to controlling the eVTOL. |

## Run it

**Terminal 1 — start the VTOL sim** (drop `HEADLESS=1` to watch in the Gazebo GUI):
```bash
cd /Users/nont/Desktop/eVTOL/sandbox/PX4-Autopilot
make px4_sitl gz_standard_vtol
```
Wait for the `pxh>` prompt and `INFO [commander] Ready for takeoff!`.

**Terminal 2 — fly the mission:**
```bash
cd /Users/nont/Desktop/eVTOL/sandbox/PX4-Autopilot
.venv/bin/python evtol_testflight/fly_evtol_mission.py
```

Options: `--alt 50` (climb height, m), `--forward 100` (North cruise, m),
`--connect udpin:0.0.0.0:14540`.

## The mission (5 steps)
1. Arm in multicopter mode, climb to **50 m**.
2. Transition to **fixed-wing** (`DO_VTOL_TRANSITION param1=4`).
3. Cruise **100 m North** as an airplane.
4. Transition back to **multicopter** (`DO_VTOL_TRANSITION param1=3`).
5. **AUTO.LAND** — vertical descent + auto-disarm.

## What to watch for in the output
- `[vtol] state -> FW` / `-> MC` — the transitions actually completing
  (read from `EXTENDED_SYS_STATE.vtol_state`).
- `GCS connection regained` — the continuous heartbeat satisfying PX4's arm check.
- `Landing detected` / `Disarmed by landing` — clean finish.

## Two things that keep it alive
- A **~1 Hz GCS heartbeat** streams the entire mission, or PX4 won't arm
  (it would sit in `UNINIT`).
- A **20 Hz position-setpoint stream** never stalls > 0.5 s, or PX4 drops OFFBOARD.
