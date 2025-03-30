import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Define your ROS 2 node
        Node(
            package='weed_detector',  # The package name
            executable='weed_detector',  # The name of the node (console script)
            name='weed_detector_node',  # The node name
            output='screen',  # Log to screen
            parameters=[{'use_sim_time': True}],  # Example of passing parameters
        ),
    ])