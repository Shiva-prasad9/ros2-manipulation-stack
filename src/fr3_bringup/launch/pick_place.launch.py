import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import yaml


def load_yaml(package_share_dir, file_path):
    full_path = os.path.join(package_share_dir, file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    fr3_moveit_config_pkg = get_package_share_directory('fr3_moveit_config')
    fr3_simulation_pkg    = get_package_share_directory('fr3_simulation')
    franka_description_pkg = get_package_share_directory('franka_description')

    # ----------------------------------------------------------------
    # 1. Include the Gazebo simulation launch (Phase 1 foundation)
    # ----------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fr3_simulation_pkg, 'launch', 'fr3_gazebo.launch.py')
        )
    )

    # ----------------------------------------------------------------
    # 2. Include the MoveIt move_group launch (Phase 2 foundation)
    # ----------------------------------------------------------------
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fr3_moveit_config_pkg, 'launch', 'move_group.launch.py')
        )
    )

    # ----------------------------------------------------------------
    # 3. Process robot description for the pick-place node
    #    MTC node needs robot_description to load the robot model
    # ----------------------------------------------------------------
    urdf_xacro_file = os.path.join(
        franka_description_pkg, 'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description = {
        'robot_description': xacro.process_file(
            urdf_xacro_file,
            mappings={
                'ros2_control':      'false',
                'use_fake_hardware': 'false',
                'gazebo':            'false',
                'hand':              'true',
            }
        ).toxml()
    }

    srdf_xacro_file = os.path.join(
        franka_description_pkg, 'robots', 'fr3', 'fr3.srdf.xacro'
    )
    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            srdf_xacro_file,
            mappings={'hand': 'true'}
        ).toxml()
    }

    kinematics_yaml = load_yaml(fr3_moveit_config_pkg, 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(fr3_moveit_config_pkg, 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml(fr3_moveit_config_pkg, 'config/ompl_planning.yaml')

    # ----------------------------------------------------------------
    # 4. Pick and place node
    #    Delayed 15s to allow Gazebo + move_group to fully initialise
    #    before MTC starts querying the planning scene
    # ----------------------------------------------------------------
    pick_place_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='fr3_manipulation',
                executable='pick_place_node',
                name='pick_place_node',
                output='screen',
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {'robot_description_kinematics': kinematics_yaml},
                    joint_limits_yaml,
                    {'ompl': ompl_planning_yaml},
                    {'use_sim_time': True},
                ],
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        move_group_launch,
        pick_place_node,
    ])