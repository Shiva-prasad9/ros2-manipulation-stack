# ros2-manipulation-stack

A full manipulation stack for the **Franka Research 3 (FR3)** arm built on ROS 2 Jazzy, Gazebo Harmonic, and MoveIt 2. Developed as a portfolio project targeting robotics engineering roles in manipulation and autonomous systems.

> **Note:** The [docs/](docs/) folder contains detailed debugging logs for each phase — five nested root causes on the GazeboSimSystem migration, the MTC stage interface-direction contract, and the ExecuteTaskSolutionCapability workaround. These document how the problems were diagnosed and resolved, not just what the final configuration looks like.

---

## Demo

![FR3 MTC pick-and-place execution](docs/phase3-demo.gif)

*FR3 arm executing a MoveIt Task Constructor pick-and-place task in Gazebo Harmonic. Three sub-trajectories: move to pre-grasp → approach → lift. Full pipeline: GenerateGraspPose + ComputeIK + allowCollisions + attach + lift → Connect → GeneratePlacePose + ComputeIK + release → return home.*

---

## Stack

| Layer | Technology |
|---|---|
| Simulator | Gazebo Harmonic (gz-sim 8) + dartsim physics |
| Hardware interface | gz_ros2_control / GazeboSimSystem |
| Joint control | JointTrajectoryController @ 50 Hz |
| Motion planning | MoveIt 2 + OMPL (RRTConnect) + KDL IK |
| Task planning | MoveIt Task Constructor (MTC) |
| Execution | ExecuteTrajectory action (custom bypass — see Phase 3 log) |
| Estimation | Custom UKF — FK process model + AprilTag measurement (Phase 4) |

---

## Packages

| Package | Role |
|---|---|
| `fr3_description` | FR3 URDF with GazeboSimSystem plugin, base anchor, initial joint positions |
| `fr3_simulation` | Gazebo world (SDF), controller config, launch files |
| `fr3_moveit_config` | MoveIt 2 config — OMPL, KDL kinematics, joint limits, time parameterization |
| `fr3_manipulation` | MTC pick-and-place node (C++) |
| `fr3_estimation` | Custom UKF node — in progress |
| `fr3_bringup` | Top-level launch wiring |

---

## Phase Status

| Phase | Description | Status | Tag |
|---|---|---|---|
| 0 | Environment & bringup — FR3 in Gazebo Harmonic | ✅ Complete | `v0.1.0` |
| 1 | ros2_control — JointTrajectoryController active | ✅ Complete | `v0.2.0` |
| 2 | MoveIt 2 — planning + execution on real physics | ✅ Complete | `v0.4.0` |
| 3 | MTC pick-and-place — task planning + execution | ⚠️ In progress | `v0.5.0` |
| 4 | Custom UKF — end-effector pose estimation | 🔧 Planned | — |

**Phase 3 current state:** MoveIt Task Constructor pipeline plans and executes successfully — arm moves through the full task sequence (pre-grasp → approach → lift → transport → place → home). Cartesian approach geometry under active refinement to achieve consistent physical contact with the target object.

**Phase 4 plan:** UKF with FK as nonlinear process model and wrist camera/AprilTag as measurement. UKF chosen over EKF specifically because FK is nonlinear — this is the core engineering justification documented in the design notes.

---

## Key Engineering Decisions

**Why GazeboSimSystem instead of mock hardware (Phase 2):** Mock hardware has no physics binding — the robot appears healthy in RViz while collapsed in Gazebo. GazeboSimSystem connects the ros2_control layer directly to the physics engine, making joint states reflect actual simulated dynamics.

**Why ExecuteTrajectory instead of ExecuteTaskSolutionCapability (Phase 3):** The `ExecuteTaskSolutionCapability` move_group plugin is not distributed in the binary Jazzy MTC apt packages. Rather than building MTC from source (which crashes WSL2 during compilation due to clock instability), we extract trajectories from the `SolutionSequence` API directly and dispatch them via the always-present `/execute_trajectory` action. See [Phase 3 problems log](docs/phase3-problems-log.md).

**Why UKF instead of EKF (Phase 4):** FK from joint angles to end-effector pose is nonlinear (involves products of rotation matrices). EKF linearises this with a Jacobian — UKF avoids the linearisation entirely by propagating sigma points through the full nonlinear model.

---

## Build & Run

```bash
# Clone with submodules (franka_description)
git clone --recurse-submodules https://github.com/Shiva-prasad9/ros2-manipulation-stack.git
cd ros2-manipulation-stack

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash

# Launch full pick-and-place
ros2 launch fr3_bringup pick_place.launch.py
```

**WSL2 requirements:** FastDDS shared memory must be disabled. Set `FASTRTPS_DEFAULT_PROFILES_FILE` to the `fastdds_no_shm.xml` file in the workspace root before launching.

---

## Debugging Logs

These document the actual failure modes encountered and how they were resolved — not just the working configuration.

- [Phase 2 — GazeboSimSystem migration](docs/phase2-problems-log.md) — 6 issues including the five nested root causes that blocked GazeboSimSystem integration
- [Phase 3 — MTC pick-and-place](docs/phase3-problems-log.md) — 9 issues including the MTC interface-direction contract, missing ExecuteTaskSolutionCapability, and finger joint stripping
- [Engineering mental model](docs/engineering-mental-model.md) — four-layer architecture, file map, debugging methodology

---

## Background

Built as a transition project from automotive control systems (function development at AVL, chassis engineering at Royal Enfield, motion cueing algorithms for Honda R&D's 9-DoF simulator) into robotics. The control systems background directly informs the UKF design in Phase 4.

