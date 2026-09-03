# Phase 3 — MTC Pick-and-Place: Problems & Debug Log

**Phase:** 3 — MoveIt Task Constructor pick-and-place
**Status:** ⚠️ In progress — `v0.5.0`

---

## Overview

Phase 3 built an autonomous pick-and-place pipeline using MoveIt Task Constructor (MTC). The MTC stage graph, planning, and execution all work. The remaining open issue is Cartesian approach geometry — the arm executes the full task sequence but the gripper approach collides with the table at the grasp pose.

---

## Issue 1 — MTC Stage Interface Direction Errors (longest-running)

**Symptom:**
```
InitStageException: cannot connect end interface of 'lift' (←) to 'move to place' (→)
```

**Root cause:** MTC solves stages **bidirectionally** — inside-out, not sequentially. `MoveRelative` inherits from `PropagatingEitherWay` and auto-determines direction from context. When placed before a `Connect` stage, MTC expects it to propagate **backward** (`←`). Forcing `FORWARD` breaks the interface contract.

**What didn't work:**
- Flat sequence without SerialContainers
- `restrictDirection(FORWARD)` — compiles but produces wrong interface
- Various SerialContainer arrangements

**What fixed it:** The correct MTC pattern uses `GenerateGraspPose` as a **Generator** inside a `SerialContainer`. The Generator anchors bidirectional solving — approach solves backward from it, lift solves forward after attach.

```
SerialContainer "pick object":
  MoveRelative "approach"      ← solved BACKWARD from grasp pose
  ComputeIK(GenerateGraspPose)   Generator — bidirectional anchor
  allowCollisions
  MoveTo "close gripper"
  ModifyPlanningScene "attach"
  MoveRelative "lift"          → solved FORWARD after attach
```

---

## Issue 2 — `GenerateGraspPose` produces no solutions

**Root cause:** `GenerateGraspPose` generates Cartesian poses — it does NOT solve IK. Without `ComputeIK` wrapping it, poses are never converted to joint configurations.

**Fix:** Always wrap with `ComputeIK`:
```cpp
auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
  "grasp pose IK", std::move(grasp_pose));
ik_wrapper->setIKFrame(grasp_frame_transform, HAND_FRAME);
pick->insert(std::move(ik_wrapper));  // insert wrapper, not grasp_pose
```

---

## Issue 3 — All grasp solutions rejected as colliding

**Symptom:** `attach object (0/8): fr3_link6 colliding with table`

**Fix:** Add `ModifyPlanningScene` to allow hand-object collisions before closing:
```cpp
s->allowCollisions(OBJECT_ID,
  task.getRobotModel()->getJointModelGroup(HAND_GROUP)
    ->getLinkModelNamesWithCollisionGeometry(), true);
```

---

## Issue 4 — `GeneratePlacePose` produces no solutions

**Root cause:** `GeneratePlacePose` needs to know how the object is currently attached to the gripper. It requires a pointer to the `attach object` stage.

**Fix:** Save a raw pointer BEFORE `std::move`:
```cpp
attach_object_stage = stage.get();  // save before move
pick->insert(std::move(stage));
// ...
place_pose->setMonitoredStage(attach_object_stage);
```

---

## Issue 5 — `ExecuteTaskSolutionCapability` not in binary Jazzy MTC

**Symptom:** `Failed to connect to the 'execute_task_solution' action server`

**Root cause:** The `ExecuteTaskSolutionCapability` move_group plugin is not distributed in the binary Jazzy MTC apt packages. Building from source crashes WSL2 due to clock instability during compilation.

**Fix — bypass `task_.execute()` entirely:**
```cpp
// Cast to SolutionSequence, iterate SubTrajectory, send via ExecuteTrajectory action
const auto* seq = dynamic_cast<const mtc::SolutionSequence*>(
  task_.solutions().front().get());
for (const auto* sub : seq->solutions()) {
  const auto* traj = dynamic_cast<const mtc::SubTrajectory*>(sub);
  // strip finger joints, send to /execute_trajectory
}
```

Key API: `storage.h` — `SolutionSequence::solutions()` returns `vector<const SolutionBase*>`, cast each to `SubTrajectory*` for `RobotTrajectory`.

---

## Issue 6 — Finger joints rejected by controller manager

**Symptom:** `Unable to identify controllers for: fr3_finger_joint1`

**Root cause:** `getRobotTrajectoryMsg()` includes ALL joints. `move_group` checks every joint against known controllers and rejects if any is uncontrolled.

**Fix:** Strip finger joints before sending:
```cpp
for (size_t i = 0; i < joint_names.size(); ++i) {
  if (joint_names[i].find("finger") == std::string::npos)
    arm_indices.push_back(i);
}
// rebuild trajectory with arm_indices only
```

---

## Issue 7 — Start state deviation between consecutive trajectories

**Symptom:** `start point deviates from current robot state more than 0.5 at joint fr3_joint4`

**Root cause:** Proportional position controller with gain=1.0 allows slight drift between trajectories. MoveIt validates start of trajectory N+1 against actual current state.

**Fix:** `allowed_start_tolerance: 0.0` in move_group — special MoveIt value that disables validation entirely.

---

## Issue 8 — Cartesian approach achieves 0% (open issue)

**Symptom:** `approach object (0/N): Achieved: 0.000000`

**Root cause:** FR3 hand geometry — `fr3_hand_tcp` is 10.34cm above `fr3_hand`. Fingers extend ~4.9cm below TCP. At the grasp pose, finger tips are inside the table collision geometry.

**Geometry:**
- Box top: z = 0.875 + 0.025 = 0.900
- Fingers at grasp: z = TCP - 0.049
- Required TCP height: z = 0.900 + 0.049 = 0.949
- `grasp_frame_transform.translation().z() = 0.05` (positions IK frame at fingertip level)
- Table top must be at least 5cm below box bottom to give finger clearance

**Status:** Under active investigation. `setIgnoreCollisions(true)` on `ComputeIK` allows IK solutions near the table but the approach Cartesian path still collision-checks and finds the gripper in collision at step 0.

---

## MTC Architecture (working pattern)

```
Task
├── CurrentState (Generator) ← pointer saved for grasp monitoring
├── MoveTo "open gripper" (JointInterpolation)
├── Connect "move to pick" (OMPL, timeout=30s)
├── SerialContainer "pick object"
│   ├── MoveRelative "approach" (Cartesian ← backward from grasp)
│   ├── ComputeIK(GenerateGraspPose) setIgnoreCollisions=true
│   ├── ModifyPlanningScene "allow collision"
│   ├── MoveTo "close gripper"
│   ├── ModifyPlanningScene "attach object" ← pointer saved for place
│   └── MoveRelative "lift" (Cartesian → forward, min=0.03m)
├── Connect "move to place" (OMPL + JointInterp, timeout=10s)
├── SerialContainer "place object"
│   ├── ComputeIK(GeneratePlacePose) monitors attach_object_stage
│   ├── MoveTo "open gripper"
│   ├── ModifyPlanningScene "forbid collision"
│   ├── ModifyPlanningScene "detach object"
│   └── MoveRelative "retreat"
└── MoveTo "return home" (OMPL)
```

---


