from __future__ import annotations

import rclpy
from gazebo_msgs.srv import SetModelConfiguration
from rclpy.node import Node


class StartupPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("startup_pose_node")

        self.model_name = self.declare_parameter("model_name", "industrial_arm").value
        self.urdf_param_name = self.declare_parameter("urdf_param_name", "robot_description").value
        self.joint_names = list(
            self.declare_parameter(
                "joint_names",
                [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_pitch_joint",
                    "wrist_yaw_joint",
                    "wrist_roll_joint",
                    "gripper_joint",
                ],
            ).value
        )
        self.joint_positions = list(
            self.declare_parameter(
                "joint_positions",
                [2.40, 0.40, -1.10, 0.75, 1.57, 0.0, 0.04],
            ).value
        )
        self.primary_service = self.declare_parameter(
            "primary_service", "/gazebo/set_model_configuration"
        ).value
        self.fallback_service = self.declare_parameter(
            "fallback_service", "/set_model_configuration"
        ).value

        self.primary_client = self.create_client(SetModelConfiguration, self.primary_service)
        self.fallback_client = self.create_client(SetModelConfiguration, self.fallback_service)

    def run(self) -> int:
        if len(self.joint_names) != len(self.joint_positions):
            self.get_logger().error("Startup pose joint-name and joint-position lengths do not match.")
            return 1

        request = SetModelConfiguration.Request()
        request.model_name = self.model_name
        request.urdf_param_name = self.urdf_param_name
        request.joint_names = list(self.joint_names)
        request.joint_positions = [float(value) for value in self.joint_positions]

        for client, service_name in (
            (self.primary_client, self.primary_service),
            (self.fallback_client, self.fallback_service),
        ):
            if not client.wait_for_service(timeout_sec=5.0):
                continue

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result()
            if response is None:
                self.get_logger().warn(f"Startup pose request returned no response from '{service_name}'.")
                continue

            if response.success:
                self.get_logger().info(
                    f"Initialized spawned robot joints via '{service_name}' before controller startup."
                )
                return 0

            self.get_logger().warn(
                f"Gazebo rejected startup pose request via '{service_name}': {response.status_message}"
            )

        self.get_logger().error("Unable to initialize the spawned robot joints in Gazebo.")
        return 1


def main() -> None:
    rclpy.init()
    node = StartupPoseNode()
    exit_code = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
