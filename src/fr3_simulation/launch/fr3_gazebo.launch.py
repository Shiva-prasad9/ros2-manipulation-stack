import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # ----------------------------------------------------------------
    # 1. Resolve package paths
    # ----------------------------------------------------------------
    franka_description_pkg = get_package_share_directory('franka_description')
    fr3_simulation_pkg     = get_package_share_directory('fr3_simulation')

    # ----------------------------------------------------------------
    # 2. Process FR3 xacro with ros2_control enabled
    #    use_fake_hardware=true → mock_components/GenericSystem
    #    No physics simulation — commands echoed back as state instantly
    #    gazebo=false → avoids franka_gazebo_bringup dependency
    # ----------------------------------------------------------------
    xacro_file = os.path.join(
        franka_description_pkg,
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'ros2_control':      'true',
            'use_fake_hardware': 'true',
            'gazebo':            'false',
            'hand':              'true',
        }
    ).toxml()

    # ----------------------------------------------------------------
    # 3. Controller config and world paths
    # ----------------------------------------------------------------
    controllers_yaml = os.path.join(
        fr3_simulation_pkg,
        'config', 'fr3_controllers.yaml'
    )
    world_file = os.path.join(
        fr3_simulation_pkg,
        'worlds', 'fr3_world.sdf'
    )

    # ----------------------------------------------------------------
    # 4. GZ_SIM_RESOURCE_PATH — parent of franka_description/
    # ----------------------------------------------------------------
    franka_share_parent = os.path.dirname(franka_description_pkg)

    return LaunchDescription([

        # ------------------------------------------------------------
        # 5. Set mesh resource path for Gazebo
        # ------------------------------------------------------------
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=franka_share_parent
        ),

        # ------------------------------------------------------------
        # 6. Launch Gazebo Harmonic with custom world
        # ------------------------------------------------------------
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),

        # ------------------------------------------------------------
        # 7. robot_state_publisher — broadcasts TF from URDF + joint states
        # ------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # ------------------------------------------------------------
        # 8. Spawn FR3 into Gazebo from /robot_description topic
        # ------------------------------------------------------------
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'fr3',
                '-topic', '/robot_description',
                '-z', '0.0'
            ],
            output='screen'
        ),

        # ------------------------------------------------------------
        # 9. Controller manager — delayed 3s for Gazebo to start
        # ------------------------------------------------------------
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[
                        {'robot_description': robot_description},
                        controllers_yaml
                    ],
                    output='screen'
                ),
            ]
        ),

        # ------------------------------------------------------------
        # 10. Spawn controllers — delayed 5s for controller_manager
        #     to fully initialise its services before spawners call them
        # ------------------------------------------------------------
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['fr3_arm_controller'],
                    output='screen'
                ),
            ]
        ),

    ])
