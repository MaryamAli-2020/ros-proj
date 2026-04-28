import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description() -> LaunchDescription:
    use_mock_qr_stream = LaunchConfiguration("use_mock_qr_stream")
    use_rviz = LaunchConfiguration("use_rviz")
    camera_topic = LaunchConfiguration("camera_topic")

    description_share = FindPackageShare("robot_description")
    moveit_share = FindPackageShare("moveit_config")
    manipulation_share = FindPackageShare("manipulation")
    bringup_share = FindPackageShare("bringup")

    description_share_path = get_package_share_directory("robot_description")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution([description_share, "urdf", "industrial_arm.urdf.xacro"]),
                    " ",
                    "use_gazebo_control:=true",
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

    common_moveit_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        {"use_sim_time": True},
    ]

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("simulation"), "launch", "gazebo.launch.py"])
        )
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("perception"), "launch", "perception.launch.py"])
        ),
        launch_arguments={
            "use_mock_qr_stream": use_mock_qr_stream,
            "camera_topic": camera_topic,
        }.items(),
    )

    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("decision_logic"), "launch", "decision.launch.py"])
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "industrial_arm"],
        output="screen",
    )

    delayed_spawn_robot = TimerAction(
        period=4.0,
        actions=[spawn_robot],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_moveit_parameters + [{"allow_trajectory_execution": True}],
    )

    manipulation_node = Node(
        package="manipulation",
        executable="manipulation_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([manipulation_share, "config", "poses.yaml"]),
            *common_moveit_parameters,
        ],
    )

    coordinator_node = Node(
        package="bringup",
        executable="coordinator_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([bringup_share, "config", "coordinator.yaml"]),
            {"auto_start": False},
        ],
    )

    startup_homing_node = Node(
        package="bringup",
        executable="startup_homing_node",
        output="screen",
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f"{description_share_path}/rviz/robot.rviz"],
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": True},
        ],
    )

    delayed_application_stack = TimerAction(
        period=1.0,
        actions=[
            move_group,
            manipulation_node,
            perception_launch,
            decision_launch,
            coordinator_node,
            rviz,
            TimerAction(period=2.0, actions=[startup_homing_node]),
        ],
    )

    start_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=0.5, actions=[joint_state_broadcaster_spawner])],
        )
    )

    start_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[TimerAction(period=1.0, actions=[arm_controller_spawner])],
        )
    )

    start_gripper_controller_after_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[TimerAction(period=1.0, actions=[gripper_controller_spawner])],
        )
    )

    start_application_after_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[delayed_application_stack],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_qr_stream", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("camera_topic", default_value="/scan_station_camera/image_raw"),
            simulation_launch,
            robot_state_publisher,
            delayed_spawn_robot,
            start_joint_state_broadcaster_after_spawn,
            start_arm_controller_after_joint_state_broadcaster,
            start_gripper_controller_after_arm_controller,
            start_application_after_gripper_controller,
        ]
    )
