import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.conditions import IfCondition
from launch.actions import Shutdown

def generate_launch_description():
    print("Launching ignition_gazebo.launch.py **********************************")

    # Get the path to the package
    pkg_path = get_package_share_directory("robot_gazebo")

    # Set the models path (models located inside 'models' folder of the package)
    models_path = Path(pkg_path, "models", "X3").as_posix()

    # Specify the path to the model file
    # drone_model_path = Path(models_path, "model.sdf").as_posix()
    
    # Set Ignition resource paths
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path + ":" + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    # Spawn the drone using ros_ign_gazebo
    spawn_drone_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'virtual_maize_field',
            '-file', Path(models_path, "model.sdf").as_posix(),
            '-name', 'X3',
            '-x', '0', '-y', '0', '-z', '1'
        ],
        output='screen'
    )



    # TimerAction to wait for a few seconds before spawning the drone
    wait_for_simulation = TimerAction(
        period=5.0,  # Wait for 5 seconds
        actions=[spawn_drone_cmd]
    )

    # # Launch Rviz for visualization
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        on_exit=Shutdown(),
    )

    # Start the ROS 2 Gazebo bridge (uncomment and configure if needed)
    bridge_params = os.path.join(
        get_package_share_directory('robot_gazebo'),
        'params',
        'ignition_ros2_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Create the launch description and add the actions
    ld = LaunchDescription()

    # Add all actions to the launch description
    ld.add_action(ign_resource_path)
    ld.add_action(wait_for_simulation)  # Wait for Gazebo to initialize
    # ld.add_action(rviz_node)
    ld.add_action(start_gazebo_ros_bridge_cmd)  # Start the ROS2-Gazebo bridge

    return ld
