from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_autonomy',
            executable='lidar_localization',
            name='lidar_localization'
        ),
        Node(
            package='robot_autonomy',
            executable='move_turtlebot',
            name='move_turtlebot'
        ),
    ])