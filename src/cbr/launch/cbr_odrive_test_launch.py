from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cbr',
            executable='cbr_odrive_can_bridge',
            name='motor1',
            parameters=[
                {'can_id': 1},
                {'joint_name': 'left_hip_joint'}
            ]
        ),
        Node(
            package='cbr',
            executable='cbr_odrive_can_bridge',
            name='motor2',
            parameters=[
                {'can_id': 1},
                {'joint_name': 'left_knee_joint'}
            ]
        )
    ])