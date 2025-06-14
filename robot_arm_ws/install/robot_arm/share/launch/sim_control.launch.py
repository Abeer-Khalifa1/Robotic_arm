from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Update these paths/names to match your setup!
    robot_name = "my_robot"  # Name of your robot in Gazebo
    urdf_path = "/home/user/robot_arm_ws/src/robot_arm/urdf/Robot_arm.urdf"  # Path to your URDF
    sdf_path = "/home/user/robot_arm_ws/src/robot_arm/models/Robot_arm.sdf"   # Path to your SDF
    bridge_config = "/home/user/robot_arm_ws/src/robot_arm/config/bridge.yaml" # Path to bridge.yaml
    moveit_config_pkg = "robot_arm_moveit_config"  # Name of your MoveIt config package

    # Launch Gazebo with your SDF model
    gazebo = ExecuteProcess(
        cmd=[
            "ros2", "launch", "ros_gz_sim", "gz_sim.launch.py",
            f"gz_args:=-r {sdf_path}"
        ],
        output="screen"
    )

    # Bridge to connect Gazebo and ROS 2 topics
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen"
    )

    # Robot State Publisher (publishes TF from URDF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": open(urdf_path).read()}],
        output="screen"
    )

    # Launch MoveIt 2
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(moveit_config_pkg),
                "launch/demo.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "robot_description": open(urdf_path).read()
        }.items()
    )

    # RViz2 with MoveIt configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory(moveit_config_pkg),
            "config/moveit.rviz"
        )],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        moveit_launch,
        rviz_node
    ])
