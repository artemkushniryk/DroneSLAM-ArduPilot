import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    print("Launching ignition_gazebo.launch.py for Drone **********************************")

    # Get package path
    pkg_path = get_package_share_directory("robot_gazebo") 
    models_path = Path(pkg_path, "models").as_posix()
    drone_model_path = Path(models_path, "X3", "model.sdf").as_posix()
    robot_desc = open(drone_model_path).read()

    # Set Ignition resource paths
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path
    )

    # Specify the world file
    #simulation_world_file_path = Path(pkg_path, "worlds/new_world/generated_world.sdf").as_posix()
    # simulation_world_file_path = Path(pkg_path, "worlds/new_world/new_world.sdf").as_posix()
    simulation_world_file_path = Path(pkg_path, "worlds/maze/model.sdf").as_posix()

    # Launch Gazebo (following nav_car style)
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch',
            'gz_sim.launch.py')),
        launch_arguments=[('gz_args', simulation_world_file_path + ' -r -v 4')]
    )

    # Spawn the Drone (using string description)
    spawn_drone_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            #'-world', 'virtual_maize_field',
            '-world', 'maze',
            '-string', robot_desc,  # Pass model as string instead of file
            '-name', 'X3',
            '-x', '0', '-y', '0', '-z', '1'
        ],
        output='screen'
    )

    robot_desc_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Bridge between Ignition and ROS 2
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': pkg_path + '/params/ignition_ros2_bridge.yaml'}],
        output='screen'
    )

    # RViz for visualization (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=['-d', os.path.join(pkg_path, 'config', 'rviz_config.rviz')],
        output="screen"
    )

    # Timer to delay model spawn until simulation is loaded
    wait_for_simulation = TimerAction(
        period=5.0,
        actions=[spawn_drone_cmd]
    )

    return LaunchDescription([
        ign_resource_path,
        gazebo_node,
        wait_for_simulation,
        bridge_node,
        robot_desc_node,
        rviz_node
    ])
