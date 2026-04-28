from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QrDecoderBridge(Node):
    """Republish zbar_ros detections onto a project-specific topic with debouncing."""

    def __init__(self) -> None:
        super().__init__("qr_decoder_bridge")

        self.barcode_topic = self.declare_parameter("barcode_topic", "/barcode").value
        self.decoded_topic = self.declare_parameter("decoded_topic", "/perception/decoded_qr").value
        self.deduplicate = self.declare_parameter("deduplicate_messages", True).value
        self.repeat_after = float(
            self.declare_parameter("republish_on_repeat_after_sec", 2.0).value
        )
        self.accepted_codes: List[str] = list(
            self.declare_parameter("accepted_codes", ["RED", "BLUE", "GREEN"]).value
        )

        self.last_message = ""
        self.last_publish_time = self.get_clock().now()

        self.publisher = self.create_publisher(String, self.decoded_topic, 10)
        self.subscription = self.create_subscription(
            String, self.barcode_topic, self.barcode_callback, 10
        )

        self.get_logger().info(
            f"Listening for zbar_ros detections on '{self.barcode_topic}' and republishing to "
            f"'{self.decoded_topic}'."
        )

    def barcode_callback(self, msg: String) -> None:
        barcode_text = msg.data.strip().upper()
        if not barcode_text:
            return

        if self.accepted_codes and barcode_text not in self.accepted_codes:
            self.get_logger().warn(
                f"Ignoring unsupported QR payload '{barcode_text}'. Allowed values: "
                f"{', '.join(self.accepted_codes)}"
            )
            return

        now = self.get_clock().now()
        should_publish = True

        if self.deduplicate and barcode_text == self.last_message:
            elapsed = (now - self.last_publish_time).nanoseconds / 1e9
            should_publish = elapsed >= self.repeat_after

        if not should_publish:
            return

        outgoing = String()
        outgoing.data = barcode_text
        self.publisher.publish(outgoing)

        self.last_message = barcode_text
        self.last_publish_time = now
        self.get_logger().info(f"Published decoded QR payload '{barcode_text}'.")


def main() -> None:
    rclpy.init()
    node = QrDecoderBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

