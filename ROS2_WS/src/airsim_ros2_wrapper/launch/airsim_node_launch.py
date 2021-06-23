from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="airsim_ros2_wrapper",
            namespace='airsim',
            executable='airsim_ros2_wrapper',
            name='airsim_ros2_wrapper'
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="ned_to_enu_pub",
            parameters="0 0 0 1.57 0 3.14 world_ned world_enu 100"
        )
    ])