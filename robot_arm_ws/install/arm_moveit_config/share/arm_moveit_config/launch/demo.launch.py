from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to your robot_arm package
    robot_arm_pkg_path = get_package_share_directory('robot_arm')
    
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "Robot_arm", 
            package_name="arm_moveit_config"
        )
        # Point to URDF in the robot_arm package
        .robot_description(
            file_path=os.path.join(robot_arm_pkg_path, "urdf", "Robot_arm.urdf.xacro"),
            mappings={}  # Add any xacro arguments if needed
        )
        # Point to SRDF in the arm_moveit_config package
        .robot_description_semantic("config/Robot_arm.srdf")
        # Configure planning pipeline
        .planning_pipelines(pipelines=["ompl"])
        # Add trajectory execution
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # Add planning scene monitor
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True
        )
        .to_moveit_configs()
    )
    
    return generate_demo_launch(moveit_config)