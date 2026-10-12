# NMPC for Quadrupedal Loco-Manipulation with Non-Negligible Manipulator Dynamics

C++ implementation of the controller from:

> **A Nonlinear MPC Framework for Loco-Manipulation of Quadrupedal Robots With Non-Negligible Manipulator Dynamics**
> Ruturaj S. Sambhus, Kapi Ketan Mehta, Ali MirMohammad Sadeghi, Basit Muhammad Imran, Jeeseop Kim, Taizoon Chunawala, Vittorio Pastore, Sujith Vijayan, Kaveh Akbari Hamed
> *IEEE Robotics and Automation Letters*, vol. 11, no. 4, pp. 4050–4057, 2026
> [Paper](https://ieeexplore.ieee.org/document/11393908) · [arXiv:2507.22042](https://arxiv.org/abs/2507.22042)

Developed at the Hybrid Dynamic Systems and Robot Locomotion Lab (HDSRL), Virginia Tech.

## Overview

The controller has two layers:

- **High level: NMPC at about 60 Hz.**
  - The quadruped trunk is modeled as a single rigid body with 12 states.
  - The manipulator keeps its full dynamics: a floating arm base plus four joints, for 20 states. Its mass matrix and gravity terms are generated with Pinocchio.
  - Six holonomic constraints attach the arm base to the trunk. The interaction wrench between them is an explicit decision variable.
  - Control inputs are the 12 ground reaction forces, the 6-D interaction wrench and 4 arm joint torques.
  - The horizon is N = 9 with Δt = 0.0167 s. The problem is built in CasADi, generated as C code and solved with IPOPT using the HSL MA57 linear solver.
- **Low level: whole-body QP at 1 kHz.** An input-output linearizing controller with a control Lyapunov function tracks the NMPC's center-of-mass trajectory and ground reaction forces on the legs. The QP is solved with iSWIFT. The arm torques come directly from the NMPC.

The controller was tested in RaiSim with a Unitree A1 + Kinova and a Unitree Go2 + Kinova. On hardware, it ran on a Go2 (15 kg) with a 4-DoF Kinova arm (4.4 kg).

## Repository layout

| Path | Contents |
|---|---|
| `src/a1_kinova_nmpc.cpp`, `include/a1_kinova_nmpc.hpp` | NMPC: reference planner, solver interface, logging |
| `SRB_NMPC_Casadi_Formulation/` | Standalone project that builds the symbolic NMPC and generates the solver code (`a1_kinova_nmpccodegen`) |
| `SRB_NMPC_Casadi_Formulation/dynamics/` | Arm dynamics from Pinocchio and CasADi (`kinova_dynamics.ipynb`), plus the generated D(q) and G(q) |
| `src/LocoWrapper.cpp`, `src/LowLevelCtrl.cpp`, `src/VirtualConstraints.cpp` | Low-level whole-body controller |
| `dynamics/` | Generated leg kinematics and dynamics |
| `optimization/iSWIFT/` | QP solver used by the low level |
| `src/A1_Kinova_*.cpp` | Simulation and hardware entry points (see [Executables](#executables)) |
| `src/JoyStick.cpp` | Gamepad interface built on JoyShockLibrary |
| `params/` | Runtime parameter files: NMPC weights, low-level gains, gait settings |
| `rsc/` | URDFs and meshes for A1, Go2 and Kinova |
| `unitree_sdk2/`, `unitree_legged_sdk/`, `kinova_arm_sdk/`, `JoyShockLibrary/` | Bundled third-party SDKs |

Some files aren't listed here, such as `src/nmpc.cpp` and `MPC_dist`. They come from earlier quadruped-only work, and the paper's results don't depend on them.

## Dependencies

Tested on Ubuntu 22.04 with GCC 11.4 and CMake 3.22.

| Dependency | Tested version | Notes |
|---|---|---|
| RaiSim (raisimLib) + raisimOgre | v1.1.7 | Simulation and visualization. Requires a RaiSim license. |
| CasADi | 3.6.7 | Built from source and installed to `/usr/local` |
| IPOPT | 3.14.17 | Built with HSL (`libcoinhsl`) |
| HSL MA57 | — | Free academic license: https://licences.stfc.ac.uk/product/coin-hsl |
| Eigen | 3.3.7 | |
| Boost | — | `timer`, `system`, `thread` |
| yaml-cpp | 0.7 | |
| LCM | 1.5 | |
| hidapi | 0.11 | `sudo apt install libhidapi-dev` (for the gamepad) |
| Pinocchio (Python) | — | Only needed to regenerate the arm dynamics. The generated files are included. |

These are bundled and need no separate install: `unitree_sdk2` (prebuilt x86_64 libraries and CycloneDDS), `unitree_legged_sdk`, the Kinova SDK, iSWIFT and JoyShockLibrary. JoyShockLibrary still has to be built once (see step 2).

**No MA57?** If your IPOPT was built with MUMPS, change `opts["ipopt.linear_solver"] = "ma57"` to `"mumps"` in `NMPC_A1_KINOVA::init()` (`src/a1_kinova_nmpc.cpp`) and in `SRB_NMPC_Casadi_Formulation/src/a1_kinova_nmpccodegen.cpp`. Solves will be slower. The timing results in the paper use MA57.

## Setup

### 1. Environment

```bash
export WORKSPACE=/path/to/raisim_workspace   # where raisimLib and raisimOgre are cloned
export LOCAL_INSTALL=/path/to/raisim_build    # install prefix for RaiSim, OGRE and raisimOgre
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$LOCAL_INSTALL/lib:/usr/local/lib
```

`CMakeLists.txt` finds RaiSim, OGRE and raisimOgre through `$LOCAL_INSTALL`. It also expects `gcc-11`/`g++-11` and CasADi under `/usr/local`; edit `CMAKE_C_COMPILER`, `CMAKE_CXX_COMPILER` and the CasADi hint there if your setup differs.

### 2. Build JoyShockLibrary (once)

```bash
cd JoyShockLibrary
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cd ..
```

This produces `JoyShockLibrary/build/JoyShockLibrary/libJoyShockLibrary.a`. The main project needs it to build, even if you won't use a gamepad.

### 3. Generate the NMPC solver

```bash
./runCodegenA1K.sh
```

This script:

1. Builds `SRB_NMPC_Casadi_Formulation`.
2. Runs `a1_kinova_nmpccodegen`, which builds the symbolic problem and compiles it to shared libraries.
3. Copies `a1_kinova_nlp_code.so` and `a1_kinova_nlp_code_f.so` into `build/`.

Compiling the generated solver can take a few minutes.

Run it again whenever you change the symbolic formulation: dynamics, constraints or horizon. The cost weights are runtime parameters, so changing them doesn't require regenerating.

### 4. Build

```bash
./cmakemake.sh
```

## Running the simulation

```bash
cd build
mkdir -p tmp
./A1_Kinova_Sim_Debug ../params/LL_w_CLF.txt ../params/Walking_params.txt ../params/A1_Kinova_NMPC_params.txt
```

Run from `build/`, because the solver is loaded from the current directory and logs are written to `build/tmp/`. Parameter files are recognized by the substrings `LL`, `Walking` and `A1_Kinova_NMPC` in their names, so keep those substrings if you rename the files.

| Time | Phase |
|---|---|
| 0–8 s | Robot settles on the ground with torques off |
| 8–12 s | Stand up |
| 12–13 s | Settle |
| 13 s to end | NMPC stance (`STAND_MPC`); trots when commanded |

The simulation ends after `SIM_TIME` seconds (40 s), set in `include/a1_kinova_loco_structs.hpp`.

By default, `A1_Kinova_Sim_Debug` loads the Go2 + Kinova model on rough terrain:

- **A1 + Kinova:** in `ExternalComm::setupRaisim()` (`src/A1_Kinova_Sim_debug.cpp`), change the URDF in `addArticulatedSystem(...)` to `A1/A1_w_kinova_fingers.urdf`.
- **Flat ground:** set `rough_terrain_en = 0` at the top of the same file.

### Executables

| Target | Source | Description |
|---|---|---|
| `A1_Kinova_Sim_Debug` | `src/A1_Kinova_Sim_debug.cpp` | **Main simulation.** Separate threads for the NMPC (about 60 Hz), the low level (1 kHz), RaiSim (1 kHz) and the gamepad. |
| `A1_Kinova_Run_Sim` | `src/A1_Kinova_Sim.cpp` | Older single-threaded simulation (A1 + Kinova) with no gamepad input. Run it with `./run_A1Kinovasim.sh`. |
| `Run_A1_Kinova_Sim_Kinova_threaded` | `src/A1_Kinova_Sim_Kinova_threaded.cpp` | Simulated quadruped, with the NMPC arm torques also sent to a physical Kinova arm |
| `Run_A1_Kinova_Exp_Debug` | `src/A1_Kinova_Exp_debug.cpp` | Hardware: Unitree Go2 (`unitree_sdk2`) + Kinova arm |
| `Joystick_test` | `src/Joystick_test.cpp` | Checks that the gamepad is detected |

## Commanding the robot

### With a gamepad (default)

The executables read a PlayStation-style controller through JoyShockLibrary, over USB or Bluetooth. Run `./Joystick_test` to confirm it's detected. On Linux you may need udev permissions for `/dev/hidraw*`.

Commands are accepted only after the stand-up phase, which ends at 13 s.

| Input | In stance (`STAND_MPC`) | While trotting |
|---|---|---|
| PS / Home | Start trotting | Stop and return to stance |
| Right stick ↕ | — | Forward velocity, up to ±0.5 m/s |
| Right stick ↔ | — | Lateral velocity, up to ±0.5 m/s |
| Left stick ↕ | Trunk pitch, limited to ±0.35 rad | — |
| D-pad ← → | Arm joint 1 velocity | Arm joint 1 velocity |
| D-pad ↑ ↓ | Arm joint 2 velocity | Arm joint 2 velocity |
| L1 / L2 | Arm joint 3 velocity | Arm joint 3 velocity |
| R1 / R2 | Arm joint 4 velocity | Arm joint 4 velocity |
| Cross (×) | Sit down | — |
| Triangle / Square | Close / open gripper (hardware only) | Close / open gripper (hardware only) |

- **Home button timing:** the button is read about every 0.5 s. Hold it until the gait switches. If you keep holding it through a second reading, it switches back.
- **Arm speed limits:** arm joint velocities are capped at 0.6 rad/s (6 rad/s for joint 4).
- **Where the mapping lives:** `JoyStick::getJoyShockState()` in `src/JoyStick.cpp` and `NMPC_A1_KINOVA::planner()` in `src/a1_kinova_nmpc.cpp`.

### Without a gamepad: scripted commands in the planner

With no gamepad connected, the robot stands up and stays in `STAND_MPC`, because nothing switches the gait. To script motions instead:

**1. Disable the gamepad thread.** In `main()` of `src/A1_Kinova_Sim_debug.cpp`, comment out both of these lines. Otherwise the thread prints `Device disconnected.` every cycle.

```cpp
// extComm.joystick = std::unique_ptr<JoyStick>(new JoyStick());
...
// extComm.JoystickThread = CreateRecurrentThreadEx("joystick_loop", 8, 16700, &ExternalComm::JoystickLoop, &extComm);
```

**2. Write the commands in `NMPC_A1_KINOVA::planner()`** (`src/a1_kinova_nmpc.cpp`). The planner takes its commands from two members:

- `joystick_state[]`: the commanded values (indices in the table below)
- `joystick_toggle_gait_`: trot on or off

Overwrite both right after the line `double trot_settle_counter = 50;`, before the gait-selection block:

```cpp
    double trot_settle_counter = 50; // (existing line)

    // ---------- Scripted commands (replace gamepad input) ----------
    for (float &v : joystick_state) v = 0.0f;                              // ignore gamepad
    const double t_ready = settlingTime + stand_duration + stand_settle;  // stand-up done (13 s)
    const double t = params.current_time;

    // Example 1: trot forward at 0.3 m/s from t_ready+2 s to t_ready+10 s, then stand
    joystick_toggle_gait_ = (t > t_ready + 2.0) && (t < t_ready + 10.0);
    if (joystick_toggle_gait_) {
        joystick_state[3] = 0.3f;    // forward velocity [m/s], body x
    }
    // -----------------------------------------------------------------
```

For other motions, replace the Example 1 block:

```cpp
    // Example 2: trot sideways to the left at 0.2 m/s
    joystick_toggle_gait_ = (t > t_ready + 2.0) && (t < t_ready + 10.0);
    if (joystick_toggle_gait_) {
        joystick_state[2] = 0.2f;    // lateral velocity [m/s], body y (+ left, - right)
    }
```

```cpp
    // Example 3: forward for 5 s, then to the right for 5 s, then stand
    joystick_toggle_gait_ = (t > t_ready + 2.0) && (t < t_ready + 12.0);
    if (joystick_toggle_gait_) {
        if (t < t_ready + 7.0) joystick_state[3] =  0.3f;   // forward
        else                   joystick_state[2] = -0.2f;   // right
    }
```

| `joystick_state` index | Meaning in the planner |
|---|---|
| 3 | Forward velocity [m/s], body frame |
| 2 | Lateral velocity [m/s], body frame, + = left |
| 1 | Pitch (stance only) |
| 6, 4, 8, 10 | Arm joint 1, 2, 3, 4 velocities [rad/s] |
| 12 | Sit down (1 = pressed) |

The planner limits velocity changes to 0.1 m/s per NMPC step and caps them at 0.5 m/s. When `joystick_toggle_gait_` turns false, the robot trots in place with a lower swing height for about 0.8 s, then switches to stance. We recommend scripted commands for simulation only.

## Tuning

| What | Where | Regenerate the solver? |
|---|---|---|
| NMPC cost weights and solver iterations | `params/A1_Kinova_NMPC_params.txt`. Only the first 17 lines are read; the commented block below them documents the order. | No |
| Low-level gains and QP weights | `params/LL_w_CLF.txt` | No |
| Stand and swing height | `params/Walking_params.txt` | No |
| Horizon length | Change `N(9)` in `SRB_NMPC_Casadi_Formulation/include/a1_kinova_nmpc_params.h` and `HORIZON` in `include/a1_kinova_loco_structs.hpp` together | Yes |
| Arm dynamics | `SRB_NMPC_Casadi_Formulation/dynamics/kinova_dynamics.ipynb` (requires Pinocchio with CasADi bindings) | Yes |

## Hardware

The hardware experiments used a Unitree Go2 with a Kinova arm mounted on the trunk. The CAD for the mount is in `kinova_arm_sdk/CAD/`. The assembly also uses a McMaster-Carr 90° angle bracket (part 8982K546), which isn't included; download its CAD from McMaster-Carr.

```bash
cd build
sudo ./Run_A1_Kinova_Exp_Debug ../params/LL_w_CLF.txt ../params/Walking_params.txt ../params/A1_Kinova_NMPC_params.txt <network_interface>
```

`<network_interface>` is the network interface connected to the Go2, for example `enp3s0`. Set the Kinova IP addresses in `src/KinovaArm.cpp` first. Run at your own risk, and always keep an emergency stop within reach.

## Related

This code builds on our distributed NMPC framework for quadruped teams: [HDSRL/SRB-CBF-NMPC](https://github.com/HDSRL/SRB-CBF-NMPC).

## Citation

```bibtex
@article{sambhus2026nmpc,
  title   = {A Nonlinear MPC Framework for Loco-Manipulation of Quadrupedal Robots With Non-Negligible Manipulator Dynamics},
  author  = {Sambhus, Ruturaj S. and Mehta, Kapi Ketan and Sadeghi, Ali MirMohammad and Imran, Basit Muhammad and Kim, Jeeseop and Chunawala, Taizoon and Pastore, Vittorio and Vijayan, Sujith and Akbari Hamed, Kaveh},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {11},
  number  = {4},
  pages   = {4050--4057},
  year    = {2026},
  doi     = {10.1109/LRA.2026.3664229}
}
```

## License

This project is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE).

## Third-party code

The bundled third-party code keeps its own license:

- `unitree_sdk2`: BSD-3-Clause
- `unitree_legged_sdk`: MPL-2.0
- `JoyShockLibrary`: MIT
- `optimization/iSWIFT`: by A. Pandala, Y. Ding and H.-W. Park (UIUC); its successor, [qpSWIFT](https://github.com/qpSWIFT/qpSWIFT), is GPL-3.0. Its `ldl` code comes from SuiteSparse.
- `kinova_arm_sdk/API`: the Kinova Gen2 SDK headers and libraries, which Kinova also distributes in [kinova-ros](https://github.com/Kinovarobotics/kinova-ros) (BSD-3-Clause). The full SDK and its documentation are available from Kinova.

RaiSim, HSL MA57, CasADi and IPOPT are not included; install them separately under their own licenses.
