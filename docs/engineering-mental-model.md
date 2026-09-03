# Engineering Mental Model 

**Purpose:** How to think about this stack

---

## The Four-Layer Mental Model

Every component belongs to one of four layers. When something breaks, identify the layer first.

```
LAYER          ONE JOB                              WHERE IT LIVES
───────────────────────────────────────────────────────────────────────
Physics        Simulate the world + hold joints     fr3_world.sdf
               under gravity                        fr3_gazebo_control.xacro

Control        Move joints on command               fr3_controllers.yaml
               + report joint states

Planning       Compute collision-free paths          ompl_planning.yaml
               + parameterize timing                 joint_limits.yaml
               + map plans to controllers            moveit_controllers.yaml
               + solve IK                            kinematics.yaml

Description    Define what the robot IS              fr3_gazebo_control.xacro
               (geometry, joints, physics binding)   franka_description (vendor)
```

---

## Which File to Build What In

```
QUESTION                                FILE / FOLDER
──────────────────────────────────────────────────────────────────────
What IS the robot?                      fr3_description/urdf/
How does the world look?                fr3_simulation/worlds/
How do I launch everything?             fr3_simulation/launch/
                                        fr3_moveit_config/launch/
How should controllers behave?          fr3_simulation/config/fr3_controllers.yaml
How should MoveIt plan?                 fr3_moveit_config/config/
Custom algorithm (UKF)?                 fr3_estimation/
Pick and place task logic?              fr3_manipulation/src/
Top-level bringup?                      fr3_bringup/launch/
```

**Rule:** Description = what the robot is. Simulation = how the world runs. MoveIt config = how planning behaves. Estimation/manipulation = your algorithms.

---

What??

**1. Explain the architecture, not the file names:**
> "The stack has four layers — Gazebo Harmonic with gz_ros2_control binding joints to dartsim physics, JointTrajectoryController for joint execution, MoveIt 2 with OMPL for motion planning, and MTC for task-level staged planning. They communicate via /joint_states, /tf, the FollowJointTrajectory action, and /monitored_planning_scene."

**2. Talk about a real debugging decision:**
> "When MoveIt kept rejecting trajectories, I isolated whether the bug was in the controller, the clock, or the planner output by publishing a manually-timed trajectory directly. That ruled out the controller — the bug was in MoveIt's time parameterization output, which needed the AddTimeOptimalParameterization response adapter."

**3. Know why each architectural decision was made:**
> "We bypassed franka_gazebo_bringup entirely and wrote our own Gazebo control xacro — because the upstream package hardcoded a dependency unavailable on Jazzy. Our layering keeps vendor code untouched."

---

## The Debugging Method

```
1. READ the exact error line (not the symptom)
2. IDENTIFY which layer it belongs to
3. FORM a hypothesis about the root cause
4. DESIGN a minimal test to confirm or rule it out
   (bypass higher layers — publish directly to controller,
    call a service directly, echo a topic)
5. FIX the confirmed root cause, not the symptom
6. DOCUMENT what broke and why
```

---

## On Memorising Specific Names

You don't memorise `robot_description_kinematics` or `AddTimeOptimalParameterization`. You memorise the pattern:

- **MoveIt reads configs under specific ROS 2 parameter namespaces.** If a YAML isn't being read, check the top-level namespace key.
- **Gazebo plugins need the full library filename.** `lib` prefix + `.so` suffix always.
- **Response adapters run after planning.** Missing time parameterization = `time between points not strictly increasing`.
- **Every environment variable has a counterpart.** `GZ_SIM_RESOURCE_PATH` for meshes, `GZ_SIM_SYSTEM_PLUGIN_PATH` for plugins.

When you forget a specific name, you look it up. What you can't look up is understanding *why it's needed* — that's what projects like this build.

---

## WSL2-Specific Workarounds

These are environment hacks, not engineering decisions. On a real robot or native Ubuntu, none of these would be needed.

| Workaround | Real robot equivalent |
|---|---|
| FastDDS SHM disabled | Not needed |
| 50 Hz controller rate | 1000 Hz standard |
| `position_proportional_gain=1.0` | franka_hardware handles this |
| `/clock` ros_gz_bridge | Less critical with RT kernel |
| `allowed_start_tolerance=0.0` | Tighter tolerance possible |
