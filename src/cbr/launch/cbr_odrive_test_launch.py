from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cbr',
            executable='cbr_odrive_can_bridge',
            name='motor1',
            parameters=[
                {'can_id': 0},
                {'joint_name': 'left_knee_joint'}
            ]
        ),
        Node(
            package='cbr',
            executable='cbr_odrive_can_bridge',
            name='motor2',
            parameters=[
                {'can_id': 1},
                {'joint_name': 'left_hip_joint'},
            ]
        ),
        Node(
            package='cbr',
            executable='cbr_odrive_can_bridge',
            name='motor3',
            parameters=[
                {'can_id': 2},
                {'joint_name': 'right_hip_joint'}
            ]
        ),
        Node(
            package='cbr',
            executable='cbr_odrive_can_bridge',
            name='motor4',
            parameters=[
                {'can_id': 3},
                {'joint_name': 'right_knee_joint'}
            ]
        ),
        # Node(
        #     package='cbr',
        #     executable='can_bridge',
        #     parameters=[
        #         {'can_bitrate': 250000}
        #     ]
        # ),
    ])