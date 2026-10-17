# Running PX4 with Simulink — Setup Guide

Goal: be able to run PX4 with Simulink in **all three** ways:
- **A. Simulink ↔ PX4 SITL over MAVLink** — Simulink commands/reads the running SITL drone.
- **B. Simulink as the plant model** — PX4 flies *your* Simulink vehicle dynamics (lockstep co-sim).
- **C. Deploy a Simulink controller into PX4** — auto-generate C++ and build it into PX4 (SITL or a Pixhawk board).

> **You are on an Apple Silicon macOS machine.** Read "The macOS reality" first — it
> changes where each workflow can run.

---

## 0. The macOS reality (read first)

MathWorks' **UAV Toolbox Support Package for PX4 Autopilots** — the piece that builds
PX4 firmware from MATLAB (used by workflows **B** and **C**) — **supports only Windows
and Linux. There is no macOS install/setup path.** Only Windows and Linux install
guides exist in the MathWorks docs.

| Workflow | Extra products beyond MATLAB/Simulink/UAV Toolbox | Runs natively on your Mac? |
|---|---|---|
| **A** MAVLink co-sim | none | ✅ Yes |
| **B** Simulink plant + PX4 Host Target | Embedded Coder, MATLAB Coder, Simulink Coder, **PX4 Support Package** | ❌ No — needs Windows/Linux |
| **C** Deploy controller to PX4 | same as B (+ a Pixhawk board for real hardware) | ❌ No — needs Windows/Linux |

**Recommended plan:**
1. Do **A on your Mac now** — it's fully native and reuses the Gazebo SITL we already have.
2. For **B and C**, use a **native x86-64 Linux (Ubuntu 22.04) or Windows 11 machine**.
   - Best for Apple Silicon: a separate Linux/Windows PC, or a **cloud x86-64 Linux VM**
     with a MATLAB license (e.g. MathWorks Cloud Center, AWS, Azure).
   - **Avoid** emulated x86 VMs on the Mac (UTM/QEMU) — MATLAB-for-Linux is x86-64 only,
     and emulating it *plus* a PX4 firmware build is painfully slow.
   - Windows path uses **WSL2** internally for the PX4 toolchain, so that machine needs
     virtualization enabled.

---

## 1. MathWorks products to license (covers all three)

| Product | A | B | C | Notes |
|---|:--:|:--:|:--:|---|
| MATLAB | ✅ | ✅ | ✅ | R2023b+ is native Apple Silicon; use the newest release. |
| Simulink | ✅ | ✅ | ✅ | |
| UAV Toolbox | ✅ | ✅ | ✅ | Provides MAVLink comms + the PX4 support-package. |
| Embedded Coder | | ✅ | ✅ | C/C++ code generation. |
| MATLAB Coder | | ✅ | ✅ | Dependency of Embedded Coder. |
| Simulink Coder | | ✅ | ✅ | Dependency of Embedded Coder. |
| Aerospace Blockset | opt | opt | opt | Handy for plant/dynamics + visualization (not required). |

These are paid products. Options: 30-day trial, annual **Home** license, **Student**
license/suite, or an organization license. **Before relying on B/C, confirm your license
actually includes Embedded Coder + the two Coders** — they're often *not* in cheaper bundles.

---

## 2. Workflow A — Simulink ↔ PX4 SITL over MAVLink (native macOS)

This is the graphical cousin of our `pymavlink` scripts: Simulink/MATLAB connects to the
**already-running** PX4 SITL (our Gazebo `x500` / `standard_vtol`) over UDP and exchanges
MAVLink messages. **No PX4 Support Package needed** — just UAV Toolbox's MAVLink support.

**Install (Mac):**
1. Install MATLAB (newest release), and in the installer tick **Simulink** + **UAV Toolbox**.
2. Launch MATLAB → `ver` → confirm `Simulink` and `UAV Toolbox` are listed.

**Connect to our SITL:**
1. Start SITL as usual: `make px4_sitl gz_x500` (or `gz_standard_vtol`).
   - PX4 streams MAVLink on **udp 14540** (offboard/onboard) and **14550** (GCS).
2. In MATLAB, use the UAV Toolbox MAVLink API (`mavlinkio`) to open a UDP client on 14540,
   read `HEARTBEAT`, and publish setpoints — mirroring `test_flight/fly_mission.py`.
3. In Simulink, use the **MAVLink Serializer / Deserializer** blocks + UDP transport to
   build the same loop as a block diagram.

**Verify A is working:** MATLAB connects, prints a `HEARTBEAT`, and (optional) arms +
sends a position setpoint that moves the drone in Gazebo. That proves the full round trip.

> Follow the MathWorks example "Control PX4 SITL / exchange MAVLink messages" for exact
> block names — they vary slightly by release.

---

## 3. Workflows B & C — on a Windows/Linux machine

Everything below happens on the **Linux/Windows** box (not the Mac).

**Install the toolchain:**
1. Install MATLAB + Simulink + UAV Toolbox + **Embedded Coder + MATLAB Coder + Simulink Coder**.
2. In MATLAB → **Add-Ons → Get Hardware Support Packages** → install
   **"UAV Toolbox Support Package for PX4 Autopilots."**
3. Run the package's **Hardware Setup** wizard:
   - Choose **PX4 Host Target** as the board, build target **`px4_sitl_default`** (this is SITL).
   - It downloads its **own** copy of PX4 source and sets up the toolchain
     (Python + GCC-ARM; on Windows it configures **WSL2**; on Linux, Ubuntu 20.04/22.04).
   - *(This PX4 copy is separate from this repo — the support package manages it.)*

**Workflow B — Simulink plant + PX4 Host Target (lockstep co-sim):**
- Open the example **"Run PX4 SITL Simulation with Quadcopter Plant in Simulink."**
- It runs PX4 Host Target (the flight stack) and a **Simulink plant model** (your vehicle
  dynamics, with MAVLink blocks) together in **lockstep** — PX4 flies *your* Simulink model.
- Swap in your own dynamics to study control/aero without Gazebo.

**Workflow C — deploy a Simulink controller into PX4:**
- Build a controller model (e.g. the "Design and Tune Quadcopter Position Controller for
  PX4" example).
- Generate code with Embedded Coder and deploy to:
  - **PX4 Host Target (SITL)** — no hardware needed, or
  - a **supported Pixhawk board** over USB (see hardware list below).
- Use **Monitor & Tune** (external mode) to adjust gains live and log signals.

**Hardware (only for real-board deploy in C):** Pixhawk 6X, Cube Orange, Pixhawk 4,
Pixhawk Cube, Pixracer, Pixhawk 1 (mRo), UVify IFO-S. Note the support package targets a
**specific PX4 firmware version** that lags PX4 mainline — don't expect this repo's exact version.

---

## 4. Setup checklist

**On the Mac (Workflow A):**
- [ ] MATLAB (newest) + Simulink + UAV Toolbox installed; `ver` confirms them
- [ ] `make px4_sitl gz_x500` running
- [ ] MATLAB `mavlinkio` connects to udp:14540 and prints a HEARTBEAT
- [ ] (stretch) Simulink model arms + moves the drone in Gazebo

**On a Windows/Linux machine (Workflows B & C):**
- [ ] MATLAB + Simulink + UAV Toolbox + Embedded Coder + MATLAB Coder + Simulink Coder
- [ ] UAV Toolbox Support Package for PX4 Autopilots installed
- [ ] Hardware Setup done → PX4 Host Target / `px4_sitl_default` builds
- [ ] Quadcopter-plant example runs in lockstep (B)
- [ ] Position-controller example deploys + Monitor & Tune works (C)
- [ ] (optional) A supported Pixhawk board for real-hardware deploy

---

## 5. Key documentation links
- PX4 support overview: https://www.mathworks.com/hardware-support/px4-autopilots.html
- Support package help: https://www.mathworks.com/help/uav/px4-spkg.html
- PX4 Host Target (SITL): https://www.mathworks.com/help/uav/px4-sitl.html
- Setup & configuration: https://www.mathworks.com/help/uav/setup-and-configuration-px4.html
- Install on Linux: https://www.mathworks.com/help/uav/px4/ug/install-support-for-px4-linux.html
- Simulink plant + SITL example: https://www.mathworks.com/help/uav/px4/ref/simulator-plant-model-example.html
- Design/tune position controller: https://www.mathworks.com/help/uav/px4/ref/design-tune-px4-quadcopter-position-controller.html
