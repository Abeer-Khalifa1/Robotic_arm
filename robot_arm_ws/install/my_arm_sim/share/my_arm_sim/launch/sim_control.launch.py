from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo from robot_arm package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('robot_arm'),
                    'launch/gazebo.launch.py'
                )
            ])
        ),

        # Start IK node
        Node(
            package='my_arm_sim',
            executable='square_ik_node',
            output='screen'
        ),

        # Start joint array splitter
        Node(
            package='my_arm_sim',
            executable='joint_array_splitter',
            output='screen'
        ),

        # Start ROS-Gazebo bridge with robot_arm's config
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(
                    get_package_share_directory('robot_arm'),
                    'config/bridge.yaml'
                )
            }],
            output='screen'
        )
    ])