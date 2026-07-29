import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def load_yaml(package_share_dir, file_path):
    """
    Small helper to load a YAML config file into a Python dict.
    MoveIt 2 nodes expect their config as ROS 2 parameters, so every
    YAML we wrote needs to be loaded and passed in as a parameters dict.
    """
    full_path = os.path.join(package_share_dir, file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    # ----------------------------------------------------------------
    # 1. Resolve package paths
    # ----------------------------------------------------------------
    franka_description_pkg = get_package_share_directory('franka_description')
    fr3_moveit_config_pkg  = get_package_share_directory('fr3_moveit_config')

    # ----------------------------------------------------------------
    # 2. Process the FR3 URDF — same mappings as Phase 1
    #    Must match the URDF used by robot_state_publisher/controllers
    #    exactly, otherwise move_group's internal robot model won't
    #    match what's actually running in the controllers.
    # ----------------------------------------------------------------
    urdf_xacro_file = os.path.join(
        franka_description_pkg,
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description = {
        'robot_description': xacro.process_file(
            urdf_xacro_file,
            mappings={
                'ros2_control':      'true',
                'use_fake_hardware': 'true',
                'gazebo':            'false',
                'hand':              'true',
            }
        ).toxml()
    }

    # ----------------------------------------------------------------
    # 3. Process the FR3 SRDF — defines planning groups, end-effector,
    #    virtual joint, and self-collision matrix (Step 2.2)
    # ----------------------------------------------------------------
    srdf_xacro_file = os.path.join(
        franka_description_pkg,
        'robots', 'fr3', 'fr3.srdf.xacro'
    )
    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            srdf_xacro_file,
            mappings={'hand': 'true'}
        ).toxml()
    }

    # ----------------------------------------------------------------
    # 4. Load all five MoveIt config YAMLs as parameter dicts
    # ----------------------------------------------------------------
    kinematics_yaml = load_yaml(fr3_moveit_config_pkg, 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(fr3_moveit_config_pkg, 'config/joint_limits.yaml')
    moveit_controllers_yaml = load_yaml(fr3_moveit_config_pkg, 'config/moveit_controllers.yaml')
    ompl_planning_yaml = load_yaml(fr3_moveit_config_pkg, 'config/ompl_planning.yaml')

    # ----------------------------------------------------------------
    # 5. Planning pipeline parameter block
    #    Tells move_group which planning plugin(s) are available and
    #    which one to use by default
    # ----------------------------------------------------------------
    planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml,
    }

    # ----------------------------------------------------------------
    # 6. Trajectory execution parameter block
    #    Tells move_group HOW to execute — which controller manager
    #    plugin to use (moveit_simple_controller_manager, set inside
    #    moveit_controllers.yaml) and execution duration tolerances
    # ----------------------------------------------------------------
    trajectory_execution_config = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.0,
    }

    # ----------------------------------------------------------------
    # 7. The move_group node itself
    #    This single node IS MoveIt 2 — it hosts the planning pipeline,
    #    the planning scene (collision world), and the execution manager
    #    that talks to fr3_arm_controller via FollowJointTrajectory.
    # ----------------------------------------------------------------
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'capabilities': 'move_group/ExecuteTaskSolutionCapability'},
            joint_limits_yaml,
            planning_pipeline_config,
            trajectory_execution_config,
            moveit_controllers_yaml,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        move_group_node,
    ])
