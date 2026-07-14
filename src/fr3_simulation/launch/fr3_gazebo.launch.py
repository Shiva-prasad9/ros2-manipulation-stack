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
    fr3_description_pkg    = get_package_share_directory('fr3_description')

    # ----------------------------------------------------------------
    # 2. Build a small wrapper xacro IN MEMORY that includes both:
    #      1. Franka's FR3 URDF (with ros2_control + gazebo=true enabled)
    #      2. OUR custom Gazebo plugin block (fr3_gazebo_control.xacro)
    #    This is the layering pattern: vendor file untouched, our
    #    additions included on top via xacro <xacro:include>.
    # ----------------------------------------------------------------
    fr3_urdf_path = os.path.join(
        franka_description_pkg, 'robots', 'fr3', 'fr3.urdf.xacro'
    )
    fr3_gazebo_control_path = os.path.join(
        fr3_description_pkg, 'urdf', 'fr3_gazebo_control.xacro'
    )

    combined_xacro = f'''<?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="fr3">
    <xacro:include filename="{fr3_urdf_path}"/>
    <xacro:include filename="{fr3_gazebo_control_path}"/>
    </robot>
    '''

    # Write the wrapper to a temp file since xacro.process_file needs a path
    combined_xacro_path = '/tmp/fr3_combined.urdf.xacro'
    with open(combined_xacro_path, 'w') as f:
        f.write(combined_xacro)

    # ----------------------------------------------------------------
    # 3. Process with gazebo=true now -- our plugin block makes this
    #    safe, since it no longer depends on franka_gazebo_bringup
    #      ros2_control=true       -> enables ros2_control URDF tags
    #      use_fake_hardware=false -> disable mock hardware
    #      gazebo=true             -> enables GazeboSimSystem plugin
    #      hand=true               -> include the FR3 gripper
    # ----------------------------------------------------------------
    robot_description = xacro.process_file(
        combined_xacro_path,
        mappings={
            'ros2_control':      'false',
            'use_fake_hardware': 'false',
            'gazebo':            'false',
            'hand':              'true',
        }
    ).toxml()

    # ----------------------------------------------------------------
    # 4. Controller config and world paths
    # ----------------------------------------------------------------
    controllers_yaml = os.path.join(
        fr3_simulation_pkg, 'config', 'fr3_controllers.yaml'
    )
    world_file = os.path.join(
        fr3_simulation_pkg, 'worlds', 'fr3_world.sdf'
    )

    # ----------------------------------------------------------------
    # 5. GZ_SIM_RESOURCE_PATH -- parent of franka_description/
    #    so Gazebo resolves model://franka_description/meshes/... URIs
    # ----------------------------------------------------------------
    franka_share_parent = os.path.dirname(franka_description_pkg)

    return LaunchDescription([

        # ------------------------------------------------------------
        # 6. Set mesh resource path for Gazebo
        # ------------------------------------------------------------
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=franka_share_parent
        ),

        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value='/opt/ros/jazzy/lib'
        ),

        # ------------------------------------------------------------
        # 7. Launch Gazebo Harmonic with our custom world
        # ------------------------------------------------------------
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),

        # ------------------------------------------------------------
        # 8. robot_state_publisher -- broadcasts TF from URDF + joint states
        # ------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': False}
            ]
        ),

        # ------------------------------------------------------------
        # 9. Spawn the FR3 into Gazebo from /robot_description topic
        #    Gazebo reads the embedded <gazebo><plugin> block in the
        #    URDF and loads gz_ros2_control automatically -- this is
        #    what binds ros2_control commands to real physics.
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

        # 9.a New addition
        # Bridge Gazebo's clock to ROS 2 /clock topic
        # Required because gz_ros2_control sets use_sim_time:=true on controllers
        # but without this bridge, no /clock topic exists and controller
        # activation times out waiting for a valid time signal
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        # ------------------------------------------------------------
        # NOTE: No separate ros2_control_node here. Gazebo's
        # gz_ros2_control plugin spawns its OWN controller_manager
        # internally, bound directly to physics simulation -- the
        # standalone controller_manager from the mock-hardware setup
        # is no longer needed or used.
        # ------------------------------------------------------------

        # ------------------------------------------------------------
        # 10. Spawn controllers into Gazebo's internal controller_manager
        #     Delayed 5s to allow Gazebo + its plugin to fully initialise
        # ------------------------------------------------------------
        TimerAction(
            period=8.0,
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