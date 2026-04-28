import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_file(package_name: str, relative_path: str) -> str:
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, encoding="utf-8") as file:
        return file.read()


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description() -> LaunchDescription:
    use_gazebo_control = LaunchConfiguration("use_gazebo_control")
    use_sim_time = LaunchConfiguration("use_sim_time")
    allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")

    description_share = FindPackageShare("robot_description")
    moveit_share = FindPackageShare("moveit_config")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution([description_share, "urdf", "industrial_arm.urdf.xacro"]),
                    " ",
                    "use_gazebo_control:=",
                    use_gazebo_control,
                    " ",
                    "controllers_file:=",
                    PathJoinSubstitution([moveit_share, "config", "ros2_controllers.yaml"]),
                ]
            ),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": load_file("moveit_config", "config/industrial_arm.srdf")
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml("moveit_config", "config/kinematics.yaml")
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml("moveit_config", "config/joint_limits.yaml")
    }
    ompl_planning_yaml = load_yaml("moveit_config", "config/ompl_planning.yaml")
    trajectory_execution = load_yaml("moveit_config", "config/trajectory_execution.yaml")
    moveit_controllers = load_yaml("moveit_config", "config/moveit_controllers.yaml")
    ompl_planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_yaml,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "allow_trajectory_execution": ParameterValue(
                    allow_trajectory_execution, value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gazebo_control", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
            move_group,
        ]
    )
