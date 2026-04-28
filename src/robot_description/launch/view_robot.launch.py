from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_package = FindPackageShare("robot_description")
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([description_package, "urdf", "industrial_arm.urdf.xacro"]),
            " ",
            "use_gazebo_control:=false",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_joint_state_gui", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description_content},
                    {"publish_frequency": 30.0},
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(use_joint_state_gui),
                output="screen",
                parameters=[
                    {"robot_description": robot_description_content},
                    {"rate": 30},
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(use_joint_state_gui),
                output="screen",
                parameters=[
                    {"robot_description": robot_description_content},
                    {"rate": 30},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution([description_package, "rviz", "robot.rviz"]),
                ],
                output="screen",
            ),
        ]
    )
