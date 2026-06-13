import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # ----------------------------------------------------------------
    # 1. Resolve package paths
    # ----------------------------------------------------------------
    franka_description_pkg = get_package_share_directory('franka_description')
    fr3_simulation_pkg     = get_package_share_directory('fr3_simulation')

    # ----------------------------------------------------------------
    # 2. Process the FR3 xacro into a plain URDF string
    # ----------------------------------------------------------------
    xacro_file = os.path.join(
        franka_description_pkg,
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    # ----------------------------------------------------------------
    # 3. Resolve the Gazebo world SDF path
    # ----------------------------------------------------------------
    world_file = os.path.join(
        fr3_simulation_pkg,
        'worlds', 'fr3_world.sdf'
    )

    # ----------------------------------------------------------------
    # 4. Locate the franka_description share directory
    #    Gazebo resolves mesh URIs like:
    #      model://franka_description/meshes/...
    #    by searching GZ_SIM_RESOURCE_PATH.
    #    We point it one level ABOVE franka_description's share folder
    #    so Gazebo finds 'franka_description' as a named model directory.
    # ----------------------------------------------------------------
    franka_share_parent = os.path.dirname(franka_description_pkg)

    return LaunchDescription([

        # ------------------------------------------------------------
        # 5. Set GZ_SIM_RESOURCE_PATH so Gazebo can resolve mesh URIs
        #    Without this, all model://franka_description/... paths
        #    fail silently — robot spawns but renders as invisible.
        # ------------------------------------------------------------
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=franka_share_parent
        ),

        # ------------------------------------------------------------
        # 6. Launch Gazebo Harmonic with our custom world
        # ------------------------------------------------------------
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),

        # ------------------------------------------------------------
        # 7. Start robot_state_publisher
        #    Reads URDF, listens to /joint_states, broadcasts TF tree
        # ------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # ------------------------------------------------------------
        # 8. Spawn the FR3 into Gazebo
        #    Reads /robot_description topic and inserts robot at z=0
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

    ])