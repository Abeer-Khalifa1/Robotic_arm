from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    sdf_path = "/home/user/robot_arm_ws/src/robot_arm/models/Robot_arm.sdf"
    urdf_path = "/home/user/robot_arm_ws/src/robot_arm/urdf/Robot_arm.urdf"
    bridge_config = "/home/user/robot_arm_ws/src/robot_arm/config/bridge.yaml"
    arm_sim_path = '/home/user/robot_arm_ws/src/robot_arm/robot_arm/square_ik_node.py'
    
    return LaunchDescription([
        # Gazebo Simulation
        ExecuteProcess(
            cmd=["gz", "sim", sdf_path],
            output="screen"
        ),
        # 🔄 ROS-Gazebo Bridge for each joint position topic
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/joint_0/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/joint_1/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/joint_2/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/joint_3/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/joint_4/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double'],
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
        ),

         # 🧠 Run the joy_control script directly with python3
        ExecuteProcess(
            cmd=['python3',arm_sim_path],
            output='screen'
        )
    ])
