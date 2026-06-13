# ros2-manipulation-stack

A ROS 2 Jazzy manipulation portfolio.

## Stack
- ROS 2 Jazzy
- Gazebo Harmonic
- MoveIt 2
- Custom UKF end-effector estimation
- Robot: Franka FR3

## Packages
| Package | Role |
|---|---|
| `fr3_description` | Custom FR3 URDF with gz_ros2_control and sensor tags |
| `fr3_simulation` | Gazebo worlds and launch files |
| `fr3_moveit_config` | MoveIt 2 configuration |
| `fr3_estimation` | Custom UKF node |
| `fr3_manipulation` | Pick-and-place orchestration |
| `fr3_bringup` | Top-level launch wiring |

## Status
🔧 Phase 0 — Environment & bringup in progress
