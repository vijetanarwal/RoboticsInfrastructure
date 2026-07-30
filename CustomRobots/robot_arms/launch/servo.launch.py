#!/usr/bin/env python3

import os
import yaml
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, "r") as f:
        return f.read()


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():

    ####################################################
    # Robot Description
    ####################################################

    package_dir = get_package_share_directory("custom_robots")

    xacro_file = os.path.join(
        package_dir,
        "models",
        "ur3",
        "ur3.urdf.xacro",
    )

    controllers_file = os.path.join(
        package_dir,
        "config",
        "ur3_controllers.yaml",
    )

    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={
            "ur_type": "ur3",
            "name": "ur",
            "prefix": "",
            "use_fake_hardware": "false",
            "sim_gazebo": "false",
            "sim_gz": "true",
            "simulation_controllers": controllers_file,
            "hmi": "false",
            "EE": "true",
            "EE_name": "robotiq_2f85",
            "camera": "false",
        },
    ).toxml()

    robot_description = {
        "robot_description": robot_description_content
    }

    ####################################################
    # Semantic Description
    ####################################################

    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "ros2srrc_ur3_moveit2",
            "config/ur3robotiq_2f85.srdf",
        )
    }

    ####################################################
    # Kinematics
    ####################################################

    kinematics_yaml = load_yaml(
        "ur3_gripper_moveit_config",
        "config/kinematics.yaml",
    )

    robot_description_kinematics = {
        "robot_description_kinematics":
            kinematics_yaml["/**"]["ros__parameters"]
    }

    ####################################################
    # Servo parameters
    ####################################################

    servo_yaml = load_yaml(
        "ur3_gripper_moveit_config",
        "config/ur_servo.yaml",
    )

    servo_params = {
        "moveit_servo": servo_yaml
    }

    ####################################################
    # Servo node
    ####################################################

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            servo_node,
        ]
    )