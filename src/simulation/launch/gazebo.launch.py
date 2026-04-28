import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    simulation_share = FindPackageShare("simulation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [simulation_share, "worlds", "pick_scan_place.world"]
                ),
            ),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value=[
                    PathJoinSubstitution([simulation_share, "models"]),
                    os.pathsep,
                    EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    world,
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                ],
                output="screen",
                on_exit=Shutdown(reason="Gazebo exited during bringup."),
            ),
            Node(
                package="simulation",
                executable="grasp_attachment_node",
                name="grasp_attachment_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([simulation_share, "config", "attachment.yaml"])
                ],
            ),
        ]
    )
