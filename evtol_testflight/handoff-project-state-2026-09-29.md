# Handoff — eVTOL / PX4 sandbox project state

**Date:** 2026-09-29
**Scope:** State audit of the whole sandbox. Next session picks up the eVTOL SITL
test flight work and the Simulink track.

> Note on location: this file lives in `evtol_testflight/` rather than `docs/`,
> because `docs/` in this repo is the upstream PX4 documentation book and should
> not be polluted with local notes.

## What was asked

"What is the state of this project?" — check any handoff or summary document
first, then the code, and report back in a form that is easy to act on.

## What was found

There is **no prior handoff or summary document**. The only files matching
`*SUMMARY.md` are upstream PX4 documentation tables of contents
(`docs/en/SUMMARY.md` and its translations) and are unrelated to this work.
This document is the first handoff for the project.

The local work is a fork of PX4-Autopilot (branch `main`) with three distinct
tracks.

### Track 1 — Generic offboard test flight (committed, superseded)

`test_flight/` — committed in `2fed31a7e3` ("test: add offboard test flight
scripts and PX4 notes", 2026-06-28). Four files: `fly_mission.py`,
`PX4_architecture_report.md`, `PX4_flow_diagram.md`, `README.md`.

This is the earlier, multicopter-oriented generation. It has been superseded by
Track 2 but is still on disk and still tracked in git.

### Track 2 — eVTOL test flight (untracked, the active work)

`evtol_testflight/` — **not committed**. Four files, last modified 2026-06-30:

- `fly_evtol_mission.py` (393 lines)
- `EVTOL_architecture.md` (174 lines)
- `EVTOL_flow_diagram.md` (136 lines)
- `README.md` (48 lines)

It is a direct evolution of `test_flight/`: `git diff --no-index
test_flight/fly_mission.py evtol_testflight/fly_evtol_mission.py` reports
193 insertions, 124 deletions.

**What the script flies** (PX4 SITL, `4004_gz_standard_vtol` airframe,
`CA_AIRFRAME=2`, 5 rotors + 3 control surfaces), driven over pymavlink on
`udpin:0.0.0.0:14540`:

1. `wait_for_position` + `wait_until_ready` — EKF2 lock, STANDBY
2. Prime the OFFBOARD setpoint stream for 2 s, then set OFFBOARD and arm
   (up to 5 attempts)
3. Climb as multicopter to target altitude
   (`ARRIVE_RADIUS_M=1.5`, `SETTLE_TIME_S=2.0`)
4. `DO_VTOL_TRANSITION` to `VTOL_STATE_FW`, streaming a forward target
5. Cruise north until within `CRUISE_REACH_M=5.0`
6. Transition back to `VTOL_STATE_MC`, stream 3 s to settle the hover
7. Set `AUTO.LAND`, poll until disarm or a 120 s timeout

The docs describe this as working, and there are real SITL logs to back that up
(see Evidence). The docs contain **no** TODO, "next", or "blocked" markers —
which means the rough edges below are undocumented, not resolved.

**Rough edges in `fly_evtol_mission.py`** (all cited in Evidence):

- No abort path. Every `wait_*` / `climb_to` / `transition` / `cruise_north`
  helper prints `TIMEOUT` and returns `False`, but the return value is never
  checked — the mission continues regardless.
- The `AUTO.LAND` mode set is fired 5 times with no ACK or result check.
- The arm loop retries blindly on any non-armed result; it does not distinguish
  `DENIED` from `TEMPORARILY_REJECTED`.
- Tuning constants are hardcoded module-level magic numbers, not validated:
  `STREAM_HZ=20.0`, `ARRIVE_RADIUS_M=1.5`, `CRUISE_REACH_M=5.0`,
  `SETTLE_TIME_S=2.0`.
- No yaw/heading control during fixed-wing cruise (`send_setpoint` is always
  called with the default `yaw=0.0`); east drift across transitions is
  untracked and uncorrected.
- No geofence, no battery or failsafe checks, no file logging — console prints
  only.

### Track 3 — Simulink integration (untracked, planning only)

`simulink_px4_setup/SETUP_GUIDE.md` — **not committed**, 141 lines, last
modified 2026-07-01. Written, never executed. It describes three workflows:

- **Workflow A — MAVLink co-simulation.** The only one that runs natively on
  macOS. Steps: MATLAB + Simulink + UAV Toolbox, `make px4_sitl gz_x500`,
  connect via `mavlinkio` to UDP 14540, verify HEARTBEAT, then arm and move the
  drone in Gazebo as a stretch goal. Status: planned, no completion record.
- **Workflows B and C** — Simulink plant with PX4 Host Target lockstep co-sim,
  and deploying a controller to PX4 firmware. Both need the UAV Toolbox Support
  Package for PX4 Autopilots, which is **Windows and Linux only**. Blocked on
  macOS by the guide's own words.

This matches the existing memory note `simulink-px4-macos-limit`.

## Environment state (verified this session)

| Thing | State |
|---|---|
| `build/px4_sitl_default/bin/px4` | Present, built 2026-06-28 23:51 |
| gz (Gazebo) | `/opt/homebrew/bin/gz`, Gazebo Sim **8.14.0** |
| pymavlink | **Missing** from system `python3`; **present** in repo `.venv` (v2.4.49) |
| Flight logs | Sessions on 2026-06-28/29, 06-30, 07-01, and **2026-09-20** (61 MB) |
| Working tree | Clean except the two untracked directories |

Two things worth knowing:

- **Run the script with `./.venv/bin/python`, not `python3`.** The system
  interpreter has no pymavlink and the script will fail at import.
- The PX4 binary was built two minutes *after* commit `eab034a0b4`
  ("fix(simulation): use add_velocity for gz Actuators message", which touched
  `GZMixingInterfaceESC.cpp` and `GZMixingInterfaceWheel.cpp`), so the binary
  **does** include that fix. However, Homebrew's gz stack was upgraded around
  2026-09-20 and the PX4 binary has not been rebuilt since June. Confirmed working on 2026-09-29: a full
  mission was flown end to end against gz 8.14.0 with the June binary, so there
  is no version skew in practice. See memory note `sitl-gz-version-skew-fix`.

## Verified run — 2026-09-29

The complete mission was re-flown this session and succeeded with no timeouts
and no errors. Roughly 90 seconds wall clock.

Commands used, in order:

```
make px4_sitl gz_standard_vtol                              # SITL + Gazebo GUI
./.venv/bin/python evtol_testflight/fly_evtol_mission.py    # the mission
```

Startup confirmed `INFO [init] Gazebo simulator 8.14.0`, `INFO [gz_bridge]
world: default, model: standard_vtol_0`, and `INFO [px4] Startup script returned
successfully`.

Mission output, abridged:

```
[climb]  reached 50 m (residual 0.28 m)
[vtol]   now in FW                              # DO_VTOL_TRANSITION param1=4
[cruise] reached North target (N=95.2 m)
[vtol]   now in MC                              # DO_VTOL_TRANSITION param1=3
[land]   touchdown confirmed, disarmed. Mission complete.
```

Two notes for whoever runs it next:

- The script's own `--alt` and `--forward` flags vary the climb altitude and
  cruise distance without editing anything.
- Once SITL is up it stays up; re-running the mission script alone re-flies it
  in seconds.

## What is still open

1. Three months of work (June 30 / July 1) sit **uncommitted** in two untracked
   directories. Nothing is backed up by git.
2. `test_flight/` is committed while its successor `evtol_testflight/` is not —
   the repo records the superseded generation and not the current one.
3. The mission script's failure handling is cosmetic: it detects timeouts and
   ignores them.
4. Workflow A of the Simulink track has never been attempted.
5. ~~Whether the June-built PX4 binary is still correct against the September gz
   8.14.0 stack.~~ **Resolved 2026-09-29** — full mission re-flown successfully.
   See "Verified run" below.

## What the next session should do first

1. Commit `evtol_testflight/` and `simulink_px4_setup/` (use the `/commit`
   skill; the project requires conventional commits with a topic scope and
   forbids Claude attribution footers — see `CLAUDE.md`).
2. Decide whether `test_flight/` should be deleted now that
   `evtol_testflight/` supersedes it.
3. Re-run the mission end to end to confirm the toolchain still works:
   `make px4_sitl gz_standard_vtol`, then
   `./.venv/bin/python evtol_testflight/fly_evtol_mission.py`.
4. If it flies, fix the abort path — make the mission stop when a phase
   times out instead of continuing into the next one.

## Dead ends — do not re-derive

- **Do not look for an existing handoff or summary.** There is none. The
  `docs/*/SUMMARY.md` files are upstream PX4 book tables of contents.
- **Do not try to install the MathWorks UAV Toolbox Support Package for PX4 on
  this Mac.** It is Windows/Linux only. Simulink Workflows B and C cannot run
  natively here; only Workflow A (MAVLink co-simulation) can.
- **Do not run the mission script with system `python3`.** Use `./.venv/bin/python`.

## Suggested skills for the next agent

- `commit` — the project's conventional-commit workflow (required by `CLAUDE.md`)
- `ponytail` — if any code is written or the script's error handling is reworked
- `i-have-adhd` — the user wants output in this shape by default
- `orchestrator` — if the next task again spans several files or tracks

## Evidence

Claims above are anchored to these citations, gathered by scouts this session:

- `evtol_testflight/README.md:3-5` — "A scripted eVTOL flight on PX4 SITL: climb
  as a multicopter → transition to fixed-wing → cruise North → transition back →
  land vertically."
- `evtol_testflight/EVTOL_architecture.md:5-6` — "written for the standard_vtol
  airframe (4004_gz_standard_vtol, CA_AIRFRAME = 2)."
- `evtol_testflight/EVTOL_flow_diagram.md:129-133` — the 1 Hz GCS heartbeat and
  20 Hz setpoint stream "make this work and bit us before".
- `evtol_testflight/fly_evtol_mission.py:89-92` — the hardcoded constants
- `evtol_testflight/fly_evtol_mission.py:272,288-289,309-310` — ignored timeout
  return values
- `evtol_testflight/fly_evtol_mission.py:339-348` — blind arm retry loop
- `evtol_testflight/fly_evtol_mission.py:373-375` — unchecked `AUTO.LAND` sets
- `simulink_px4_setup/SETUP_GUIDE.md:16-17` — "supports only Windows and Linux.
  There is no macOS install/setup path."
- `simulink_px4_setup/SETUP_GUIDE.md:119-122` — Workflow A verification checklist

## Related memory notes

`sitl-gz-version-skew-fix`, `px4-arm-needs-gcs-heartbeat`,
`simulink-px4-macos-limit` — all three were independently corroborated by this
audit.
