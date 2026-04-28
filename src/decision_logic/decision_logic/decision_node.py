from __future__ import annotations

from typing import Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from workflow_interfaces.msg import BinTarget


class DecisionNode(Node):
    """Translate QR payloads into configurable bin identities and placement poses."""

    def __init__(self) -> None:
        super().__init__("decision_node")

        self.qr_topic = self.declare_parameter("qr_topic", "/perception/decoded_qr").value
        self.bin_target_topic = self.declare_parameter(
            "bin_target_topic", "/decision_logic/bin_target"
        ).value
        self.default_bin_id = self.declare_parameter("default_bin_id", "bin_a").value
        self.qr_to_bin: Dict[str, str] = {
            "RED": self.declare_parameter("qr_to_bin.RED", "bin_a").value,
            "BLUE": self.declare_parameter("qr_to_bin.BLUE", "bin_b").value,
            "GREEN": self.declare_parameter("qr_to_bin.GREEN", "bin_c").value,
        }
        self.bin_poses: Dict[str, List[float]] = {
            "bin_a": list(
                self.declare_parameter(
                    "bin_pose.bin_a", [0.35, 0.45, 1.02, 0.0, -0.7071068, 0.0, 0.7071068]
                ).value
            ),
            "bin_b": list(
                self.declare_parameter(
                    "bin_pose.bin_b", [0.55, 0.45, 1.02, 0.0, -0.7071068, 0.0, 0.7071068]
                ).value
            ),
            "bin_c": list(
                self.declare_parameter(
                    "bin_pose.bin_c", [0.75, 0.45, 1.02, 0.0, -0.7071068, 0.0, 0.7071068]
                ).value
            ),
        }

        self.publisher = self.create_publisher(BinTarget, self.bin_target_topic, 10)
        self.subscription = self.create_subscription(String, self.qr_topic, self.qr_callback, 10)

        self.get_logger().info(
            f"Decision node ready. Routing payloads from '{self.qr_topic}' to '{self.bin_target_topic}'."
        )

    def qr_callback(self, msg: String) -> None:
        qr_text = msg.data.strip().upper()
        bin_id = self.qr_to_bin.get(qr_text, self.default_bin_id)

        if bin_id not in self.bin_poses:
            self.get_logger().error(
                f"No pose configured for bin '{bin_id}'. Check decision_logic/config/bin_map.yaml."
            )
            return

        pose_values = self.bin_poses[bin_id]
        outgoing = BinTarget()
        outgoing.qr_text = qr_text
        outgoing.bin_id = bin_id
        outgoing.pose = self.pose_from_values(pose_values)
        self.publisher.publish(outgoing)

        self.get_logger().info(f"Mapped QR payload '{qr_text}' to target bin '{bin_id}'.")

    def pose_from_values(self, values: List[float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(values[0])
        pose.pose.position.y = float(values[1])
        pose.pose.position.z = float(values[2])
        pose.pose.orientation.x = float(values[3])
        pose.pose.orientation.y = float(values[4])
        pose.pose.orientation.z = float(values[5])
        pose.pose.orientation.w = float(values[6])
        return pose


def main() -> None:
    rclpy.init()
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
