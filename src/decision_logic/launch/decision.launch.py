from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="decision_logic",
                executable="decision_node",
                name="decision_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("decision_logic"), "config", "bin_map.yaml"]
                    )
                ],
            )
        ]
    )
