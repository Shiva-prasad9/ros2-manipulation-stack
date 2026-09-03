# Phase 2 — GazeboSimSystem Migration: Problems & Debug Log

**Phase:** 2 — MoveIt 2 integration with real Gazebo physics
**Status:** ✅ Complete — `v0.4.0`

---

## Overview

Phase 2 migrated from mock hardware to `GazeboSimSystem` — the plugin that binds ros2_control directly to Gazebo's physics engine. Five separate blocking issues masked each other sequentially across two sessions.

---

## Issue 1 — `franka_gazebo_bringup` not available for Jazzy

**Symptom:** `Package 'franka_gazebo_bringup' not found`

**Fix:** Wrote our own `fr3_gazebo_control.xacro` defining the `<ros2_control>` block directly. No upstream dependency needed.

---

## Issue 2 — Wrong plugin filename

**Symptom:** `Failed to load plugin: gz_ros2_control-system`

**Root cause:** Plugin binary requires full Linux shared library name.

**Fix:**
```xml
<plugin filename="libgz_ros2_control-system.so"
        name="gz_ros2_control::GazeboSimSystem">
```

---

## Issue 3 — Missing `GZ_SIM_SYSTEM_PLUGIN_PATH`

**Fix:** Added to launch file:
```python
env={'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib'}
```

Note: `GZ_SIM_RESOURCE_PATH` (meshes) and `GZ_SIM_SYSTEM_PLUGIN_PATH` (plugin binaries) are separate — both must be set.

---

## Issue 4 — No physics step size → robot collapses

**Fix:** Added to `fr3_world.sdf`:
```xml
<physics name="1ms" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

---

## Issue 5 — WSL2 clock instability → controller activation timeout

**Symptom:** `/clock` topic showed `min: -28s max: +27s` — jumping backwards and forwards.

**Fix:**
1. Added `/clock` bridge via `ros_gz_bridge`
2. Set `allowed_start_tolerance: 0.0` in move_group (special MoveIt value — disables check, not "zero tolerance")

---

## Issue 6 — Robot healthy in RViz, collapsed in Gazebo

**Root cause:** RViz reads from `/tf` → `robot_state_publisher` → `/joint_states`. Mock hardware publishes commanded positions regardless of physics. RViz showed commanded state, not actual simulation.

**This masked all the above issues** for two sessions.

**Key diagnostic:**
```bash
gz topic -e -t /world/fr3_world/stats
```

**Lesson:** In a simulated system, RViz showing a healthy robot means nothing until you verify the physics engine is actually stepping.

---

## Final Working Configuration (WSL2)

| Parameter | Value | Reason |
|---|---|---|
| `position_proportional_gain` | `1.0` | Higher causes oscillation under dartsim |
| Controller update rate | `50 Hz` | No RT kernel on WSL2 |
| `allowed_start_tolerance` | `0.0` | Arm drifts slightly between trajectories |
| FastDDS shared memory | Disabled | WSL2 SHM transport broken |
| `joint_limits.yaml` namespace | `robot_description_planning` | MoveIt requires this wrapper — bare top-level key silently ignored |
| Response adapter | `AddTimeOptimalParameterization` | Without this, trajectory waypoints have no timestamps and controller rejects every goal |
