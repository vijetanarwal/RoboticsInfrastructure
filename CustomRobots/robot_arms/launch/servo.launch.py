import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "ur3",
            package_name="ur3_gripper_moveit_config",
        ).to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml(
        "ur3_gripper_moveit_config",
        "config/ur_servo.yaml"
    )

    servo_params = {
        "moveit_servo": servo_yaml
    }

    joint_limits_yaml = load_yaml(
        "ros2srrc_robots",
        "ur3/config/joint_limits.yaml"
    )

    pilz_cartesian_limits = load_yaml(
        "ros2srrc_robots",
        "ur3/config/pilz_cartesian_limits.yaml"
    )

    combined_planning = {
        "robot_description_planning": {
            **joint_limits_yaml,
            **pilz_cartesian_limits,
        }
    }

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            combined_planning,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            servo_node,
        ]
    )
