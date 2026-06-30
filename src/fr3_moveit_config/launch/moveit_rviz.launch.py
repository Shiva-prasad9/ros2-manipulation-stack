import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def load_yaml(package_share_dir, file_path):
    full_path = os.path.join(package_share_dir, file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    franka_description_pkg = get_package_share_directory('franka_description')
    fr3_moveit_config_pkg  = get_package_share_directory('fr3_moveit_config')

    # ----------------------------------------------------------------
    # Same URDF/SRDF processing as move_group.launch.py — RViz needs
    # its own copy of robot_description and robot_description_semantic
    # to render the robot model and planning scene correctly.
    # ----------------------------------------------------------------
    urdf_xacro_file = os.path.join(
        franka_description_pkg, 'robots', 'fr3', 'fr3.urdf.xacro'
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

    # ----------------------------------------------------------------
    # RViz node with MoveIt parameters injected directly.
    # This is what makes the MotionPlanning display actually connect —
    # a bare 'ros2 run rviz2 rviz2' has none of these parameters set,
    # so the MotionPlanning plugin can't initialize its planning scene
    # monitor properly even though move_group itself is healthy.
    # ----------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
        ],
    )

    return LaunchDescription([
        rviz_node,
    ])
