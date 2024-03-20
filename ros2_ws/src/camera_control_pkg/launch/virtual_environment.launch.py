import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths
    
    world_file_path = "../gazebo_world/gazebo_worlds.world"

    # Launch Gazebo with the world file
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path],
        output='screen'
    )

    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # Launch your ROS nodes
    depth_processor_node = Node(
        package='camera_control_pkg',
        executable='depth_processor',
        output='screen'
    )
    
    key_node = Node(
        package='keyboard_pkg',
        executable='key_node',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
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
