from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sdf_path = "/home/user/robot_arm_ws/src/robot_arm/models/Robot_arm.sdf"
    urdf_path = "/home/user/robot_arm_ws/src/robot_arm/urdf/Robot_arm.urdf"
    bridge_config = "/home/user/robot_arm_ws/src/robot_arm/config/bridge.yaml"

    return LaunchDescription([

        # Gazebo Simulation
        ExecuteProcess(
            cmd=["gz", "sim", sdf_path],
            output="screen"
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

        # ROS-Gazebo Bridge (Updated)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": bridge_config}],
            output="screen"
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": open(urdf_path).read(),
                "use_sim_time": True
            }],
            output="screen"
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[{"use_sim_time": True}]
        )
    ])