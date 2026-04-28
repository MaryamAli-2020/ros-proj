import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_file(package_name: str, relative_path: str) -> str:
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, encoding="utf-8") as file:
        return file.read()


def generate_launch_description() -> LaunchDescription:
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    description_share = FindPackageShare("robot_description")
    moveit_share = FindPackageShare("moveit_config")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([description_share, "urdf", "industrial_arm.urdf.xacro"]),
            " ",
            "use_gazebo_control:=false",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file("moveit_config", "config/industrial_arm.srdf")
    }

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_share, "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            "use_gazebo_control": "false",
            "use_sim_time": "false",
            "allow_trajectory_execution": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_joint_state_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    robot_description,
                    {"publish_frequency": 30.0},
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(use_joint_state_gui),
                output="screen",
                parameters=[
                    robot_description,
                    {"rate": 30},
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(use_joint_state_gui),
                output="screen",
                parameters=[
                    robot_description,
                    {"rate": 30},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
                output="screen",
            ),
            move_group,
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution([description_share, "rviz", "robot.rviz"]),
                ],
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {"use_sim_time": False},
                ],
            ),
        ]
    )
