import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths
    world_file_path = "../gazebo_world/gazebo_worlds.world"

    # Launch Gazebo with the world file in a separate terminal
    gazebo_process = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'gazebo', '--verbose', world_file_path],
        output='screen'
    )

    # Launch RViz2 in a separate terminal
    rviz_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'rviz2'],
        output='screen'
    )

    # Launch your ROS nodes in separate terminals
    depth_processor_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'camera_control_pkg', 'depth_processor'],
        output='screen'
    )
    
    key_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'keyboard_pkg', 'key_node'],
        output='screen'
    )

    joy_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add actions to the launch description
    ld.add_action(gazebo_process)
    ld.add_action(rviz_node)
    ld.add_action(depth_processor_node)
    ld.add_action(joy_node)
    ld.add_action(key_node)

    return ld
