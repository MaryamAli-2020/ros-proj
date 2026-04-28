from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_mock_qr_stream = LaunchConfiguration("use_mock_qr_stream")
    camera_topic = LaunchConfiguration("camera_topic")
    config_file = PathJoinSubstitution([FindPackageShare("perception"), "config", "perception.yaml"])

    barcode_reader_mock = Node(
        package="zbar_ros",
        executable="barcode_reader",
        name="barcode_reader",
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[("image", "/demo_qr/image_raw"), ("barcode", "/barcode")],
        condition=IfCondition(use_mock_qr_stream),
    )

    barcode_reader_camera = Node(
        package="zbar_ros",
        executable="barcode_reader",
        name="barcode_reader",
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[("image", camera_topic), ("barcode", "/barcode")],
        condition=UnlessCondition(use_mock_qr_stream),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_qr_stream", default_value="true"),
            DeclareLaunchArgument("camera_topic", default_value="/scan_station_camera/image_raw"),
            Node(
                package="perception",
                executable="mock_qr_stream",
                name="mock_qr_stream",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(use_mock_qr_stream),
            ),
            barcode_reader_mock,
            barcode_reader_camera,
            Node(
                package="perception",
                executable="qr_decoder_bridge",
                name="qr_decoder_bridge",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
