# Design: isolated simulation runtimes for Isaac Sim, Pegasus and Isaac Lab

| | |
|---|---|
| Status | **Proposed**, not yet implemented |
| Scope | Simulation stack: Isaac Sim, Pegasus Simulator, Isaac Lab, PX4 SITL, ROS 2 |
| Related | [pegasus.md](../simulation/isaac-sim/pegasus.md), [isaac_lab.md](../auto_planning/isaac_lab.md), [interfaces](../interfaces/readme.md) |

## 1. Context

Drone Ranger needs all three of these:

- **Isaac Sim**: the rendering and physics platform.
- **Pegasus Simulator**: multirotors flown by the real PX4 autopilot (SITL), with ROS 2 sensor output. Used for validation close to the real system.
- **Isaac Lab**: massively parallel reinforcement learning. Used for training policies.

Installing Isaac Lab into the same Isaac Sim that Pegasus uses broke the Pegasus setup.
`isaaclab.sh -p -m pip install ...` wrote straight into `/home/rangerlab/isaac-sim/kit/python`, and afterwards `pip check` reported conflicts that were not there before:

```
ipython 9.17.1 requires psutil>=7, but you have psutil 5.9.8
dex-retargeting 0.4.6 requires lxml>=5.2.2, but you have lxml 4.9.4
nvidia-srl-base 1.3.0 requires docstring-parser==0.16, but you have docstring-parser 0.18.0
```

This is not a one-off mistake. The same kind of breakage happens even when Isaac Sim 5.1.0, Pegasus v5.1.0 and Isaac Lab v2.3.0 are exactly the versions that are meant to go together.

## 2. Root cause

Isaac Sim is not a library. It is a **runtime platform** with:

- its own Python 3.11 interpreter;
- its own curated packages, in `site-packages` **and** in many extensions' `pip_prebundle` folders, which pip does not know about;
- its own extension system and app definitions (`.kit` files).

Pegasus and Isaac Lab are two **applications on that platform**. Each assumes it owns the platform:

| Shared resource | Pegasus | Isaac Lab |
|---|---|---|
| Python environment | `pip install -e`; its `setup.py` also **edits Isaac Sim's `apps/isaacsim.exp.base.kit`** | `isaaclab.sh --install` adds torch, RL libraries and their transitive dependencies |
| Kit app / extensions | Isaac Sim's standard app; depends on deprecated `omni.isaac.core`, `omni.isaac.dynamic_control`, `isaacsim.replicator.agent.core` | its own `isaaclab.python*.kit`, which loads none of these |
| Simulation loop | its own `World`; a few vehicles, each in lockstep with an external PX4 process, per-vehicle forces on the CPU side | its own `SimulationContext`; thousands of cloned envs, batched GPU tensors, env-manager resets |
| Process environment | needs ROS 2 (Python 3.10) and PX4 running alongside | no ROS |

These are shared, mutable resources with no arbiter. The resulting conflicts sit in four layers:

1. **Packages.** pip resolves transitive dependencies to whatever is newest *at install time* and keeps no lock file.
   Matching the three top-level versions does not pin the hundreds of packages below them, so two installs a month apart differ.
2. **App and extensions.** The two applications use different `.kit` apps and different extension sets, and Pegasus modifies a platform file.
3. **Execution model.** Pegasus's `World` and Isaac Lab's `SimulationContext` subclass the same `isaacsim.core.api` singleton, so only one can exist per process.
   More fundamentally, "a few vehicles, real autopilot, near real time" and "thousands of envs, GPU only, as fast as possible" are different simulation paradigms.
   They should not share a process.
4. **System environment.** ROS 2 Humble's Python 3.10 paths, an active conda env, and `LD_LIBRARY_PATH` leak from the shell into the platform.

On top of that, the three projects release independently. Pegasus trails Isaac Sim releases and still uses APIs that Isaac Sim is deprecating, so "matching versions" is a moving target.

## 3. Goals and non-goals

**Goals**

- Pegasus, Isaac Lab and ROS 2 can all be installed on one machine without affecting each other.
- An environment can be rebuilt bit-for-bit on another machine or by another person.
- A policy trained in Isaac Lab can be evaluated in Pegasus with real PX4, and then deployed on the drone, through one consistent chain.
- Upgrades are deliberate and tested, never a side effect of installing something else.

**Non-goals**

- Running Pegasus and Isaac Lab in the same process.
- Running real PX4 inside massively parallel training (see §8).

## 4. Design principle

> **Treat Isaac Sim as an immutable platform image.**
> **Give each application its own frozen runtime.**
> **Integrate runtimes only through data and interfaces, never through a shared process or a shared Python environment.**

## 5. Architecture

```
            ┌──────────── Contract layer (versioned in git, mounted read-only) ────────────┐
            │ Scene assets      backyard.usd, ... (Z-up, metres)                            │
            │ Vehicle model     mass / inertia / thrust curve / camera intrinsics+extrinsics │
            │ Interfaces        topics / frames / units (docs/interfaces)                    │
            │ Policy artifact   policy.onnx + observation / action spec                      │
            └──────────┬─────────────────────────┬──────────────────────────────────────────┘
                       │                         │
  ┌────────────────────▼────────────┐  ┌─────────▼──────────────────────────────┐
  │ Runtime B  "train"               │  │ Runtime A  "sitl"                       │
  │ Isaac Sim 5.1 + Isaac Lab 2.3    │  │ Isaac Sim 5.1 + Pegasus 5.1.0           │
  │ + RL libraries (locked)          │  │ + PX4 v1.14.3                           │
  │ parallel training, no PX4/ROS    │  │ few vehicles, real autopilot, lockstep  │
  └───────────────┬─────────────────┘  └───────┬──────────────────▲──────────────┘
                  │ exports policy.onnx         │ MAVLink / uXRCE-DDS / ROS 2 (network)
                  │                             │                  │
                  │                ┌────────────▼──────────────────┴─────────────┐
                  └──────────────► │ Runtime C  "ros"                             │
                                   │ ROS 2 Humble, MicroXRCEAgent, policy node    │
                                   │ policy node ──► /fmu/in/* (offboard)          │
                                   │ the same node later runs on the Jetson       │
                                   └──────────────────────────────────────────────┘
```

### 5.1 Runtimes

| Runtime | Contents | Used for | Talks to others via |
|---|---|---|---|
| **A `sitl`** | Isaac Sim 5.1.0, Pegasus v5.1.0, PX4 v1.14.3 (`px4_sitl_default`) | SITL validation, sensor data generation, demos | MAVLink (QGC :14550), uXRCE-DDS (:8888), ROS 2 DDS topics |
| **B `train`** | Isaac Sim 5.1.0, Isaac Lab v2.3.x, RL libraries, Drone Ranger tasks | policy and perception training | files: reads the contract layer, writes policy artifacts |
| **C `ros`** | ROS 2 Humble, `px4_msgs` (`release/1.14`), Micro-XRCE-DDS-Agent v2.4.2, policy inference | bridging, policy execution, recording | ROS 2 DDS (same `ROS_DOMAIN_ID`, Fast DDS) |

Runtimes A and B never run inside each other. C never imports anything from A or B.
The Python 3.10 (ROS) vs 3.11 (Isaac Sim) problem disappears structurally, with no `isaac_run`-style workarounds needed.

### 5.2 Realisation: container images (preferred)

- **Base image**: NVIDIA's `nvcr.io/nvidia/isaac-sim:5.1.0`.
- **B**: Isaac Lab's own `docker/Dockerfile.base`, which already builds `FROM nvcr.io/nvidia/isaac-sim:5.1.0`. Add a Drone Ranger layer with the tasks and locked RL dependencies.
- **A**: `FROM nvcr.io/nvidia/isaac-sim:5.1.0`, plus Pegasus at tag `v5.1.0` (`pip install -e`), plus PX4 v1.14.3 built in the image.
- **C**: `ros:humble` plus `px4_msgs`, the XRCE agent and the policy node. This is also the base for the Jetson image (arm64 build).
- A and B share the Isaac Sim base layers, so **Isaac Sim is stored on disk once**.
- GPU access via the NVIDIA Container Toolkit. GUI via Isaac Sim livestream (WebRTC client), or X11 forwarding for local use.
- Shared directories mounted into the containers: `contracts/` (read-only), `assets/` (read-only), `artifacts/` (policies, logs, bags).

**Fallback without containers**: two separately extracted Isaac Sim 5.1.0 directories, for example `~/isaacsim-sitl` (Pegasus) and `~/isaacsim-train` (Isaac Lab).
Make each directory read-only after setup, and commit its lock file (§6). This costs a second full copy of Isaac Sim on disk.

## 6. Rules

1. **No ad-hoc changes inside a runtime.** No interactive `pip install`, no editing of platform files. Every change goes into the build recipe (Dockerfile or setup script) and the runtime is rebuilt.
2. **Every runtime has a lock file**, committed to this repo.
   The list must include the `pip_prebundle` packages that `pip freeze` cannot see, for example from
   `find <isaacsim> -type d -name "*.dist-info" -printf '%f\n' | sort`.
3. **A runtime is identified by its image tag**, for example `drone-ranger/sitl:stack-2026.10`. Experiment notes record the tags they ran on.
4. **System dependencies stay outside Isaac Sim runtimes.** ROS 2 and PX4 are reached over the network, never imported into Isaac Sim's Python.
5. **Cross-runtime data flows only through the contract layer** (§7).

## 7. Contract layer

This layer is what makes the runtimes *agree*, not merely coexist.

| Contract | Single source of truth | Consumers |
|---|---|---|
| **Vehicle model** | one YAML (e.g. `config/vehicle/x500.yaml`): mass, inertia, arm geometry, rotor thrust and torque curves, motor time constants, drag, camera intrinsics/extrinsics, depth range and noise | generator for Pegasus `MultirotorConfig` (A), generator for the Isaac Lab multirotor config (B), TF and camera_info (C, real drone) |
| **Action interface** | policy outputs **PX4 offboard setpoints** (velocity or attitude/body-rate + thrust), not motor commands | B trains against a controller that mimics PX4's loop; C forwards to `/fmu/in/*`; the real PX4 executes |
| **Observation interface** | topic names, frames and units in [docs/interfaces](../interfaces/readme.md) | A publishes them in sim; the real drone publishes them on hardware; the policy node consumes them |
| **Scene assets** | USD files in a versioned asset folder (Z-up, metres, static triangle-mesh colliders), e.g. from [colmap_pipeline.md](../reconstruction/colmap_pipeline.md) | loaded by both A and B |
| **Policy artifact** | `policy.onnx` + `policy_spec.yaml` (observation layout, normalisation, action type, control rate) | produced by B, run by C in SITL and on the Jetson |

Without a shared vehicle model and action interface, the drone in training and the drone in SITL are different aircraft. A policy that works in one then fails in the other for reasons unrelated to learning.

## 8. Workflow

```
B: train in Isaac Lab ──► policy.onnx + spec ──► C+A: run policy node against Pegasus + real PX4 (SITL)
        ▲                                                  │  metrics: success, tracking error, sim-to-sim gap
        └──────────────── iterate ◄──────────────────────── ┘
                                                           │
                                                           ▼
                                        C on Jetson: same node, real PX4, real sensors
```

**Design trade-off: no real PX4 in the training loop.**
Thousands of parallel PX4 SITL processes in lockstep are not practical.
Training therefore uses a GPU-side controller that approximates PX4's control loops (published prior work does this), and the real autopilot enters only in runtime A.
This is a deliberate decision. It is why the action interface in §7 must match PX4's offboard setpoints.

## 9. Stack versioning and upgrades

A **stack version** pins everything together, for example:

| `stack-2026.10` | |
|---|---|
| Isaac Sim | 5.1.0 |
| Pegasus Simulator | v5.1.0 |
| Isaac Lab | v2.3.x |
| PX4-Autopilot | v1.14.3 |
| px4_msgs | `release/1.14` |
| Micro-XRCE-DDS-Agent | v2.4.2 |
| ROS 2 | Humble |

Upgrade rules:

- Upgrade the whole stack together. Never upgrade one component alone.
- The slowest component sets the pace. In practice this is Pegasus: Isaac Sim moves only when a Pegasus release supports it.
- A new stack version passes a smoke test in every runtime before use:
  - A: Pegasus example with PX4 takeoff, and `/fmu/out/vehicle_local_position` plus camera topics visible from C;
  - B: a short training run;
  - C: the policy node in SITL.

## 10. Risks and open questions

| Item | Notes |
|---|---|
| Pegasus in containers is untested upstream | Pegasus is officially tested only on the workstation install. Validate GUI or livestream, PX4 auto-launch inside the container, and the MAVLink/DDS ports before adopting. Fall back to separate directories (§5.2) if needed. |
| Disk space | Containers share the Isaac Sim base layers. The directory fallback needs a second full Isaac Sim copy. |
| Deprecated APIs in Pegasus | `omni.isaac.core` / `dynamic_control` will disappear in future Isaac Sim versions. This blocks stack upgrades until Pegasus migrates (also a possible upstream contribution). |
| PX4-approximating controller fidelity | The sim-to-sim gap between B's controller and real PX4 in A must be measured, not assumed. |
| GUI and remote use | Livestream needs ports (TCP 49100 / UDP 47998) and a client; this affects remote GPU machines. |

## 11. Migration from the current setup

1. Keep the current `/home/rangerlab/isaac-sim` untouched as evidence until runtime A works.
2. Build runtime A (Pegasus) first, from a clean Isaac Sim 5.1.0, and pass the A smoke test.
3. Build runtime B (Isaac Lab) separately and move the existing Drone Ranger Isaac Lab code (e.g. `drone_arl`) into its build.
4. Build runtime C and move ROS 2 / XRCE / policy code into it.
5. Create the contract files (vehicle YAML first), then the generators for A and B.
6. Update the setup guides to this design. [pegasus.md](../simulation/isaac-sim/pegasus.md) §12 and [isaac_lab.md](../auto_planning/isaac_lab.md) currently describe a *shared* Isaac Sim install and must be revised.
