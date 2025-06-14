from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', 'urdf/Robot_arm.urdf', '-entity', 'robot_arm'],
            output='screen'
        ),
        Node(
            package='robot_arm',
            executable='square_ik_node.py',
            name='square_ik_node',
            output='screen'
        )
    ])