import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_ros_rclpy_pkg',
            executable='helloworld_publisher',
            name='publisher_node'
        ),
        Node(
            package='my_first_ros_rclpy_pkg',
            executable='helloworld_subscriber',
            name='subscriber_node'
        ),
    ])