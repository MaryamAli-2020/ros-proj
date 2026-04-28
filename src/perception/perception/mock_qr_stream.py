from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MockQrStream(Node):
    """Publish a deterministic QR image for oral demos and automated verification."""

    def __init__(self) -> None:
        super().__init__("mock_qr_stream")

        self.qr_text = self.declare_parameter("qr_text", "RED").value
        self.output_topic = self.declare_parameter("output_topic", "/demo_qr/image_raw").value
        self.frame_id = self.declare_parameter("frame_id", "camera_optical_frame").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 2.0).value)
        self.image_size_px = int(self.declare_parameter("image_size_px", 480).value)
        self.cached_image = None

        self.publisher = self.create_publisher(Image, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_image)

        self.get_logger().info(
            f"Publishing synthetic QR images for payload '{self.qr_text}' to '{self.output_topic}'."
        )

    def publish_image(self) -> None:
        if self.cached_image is None:
            try:
                import qrcode
            except ImportError as exc:
                self.get_logger().error(
                    "python3-qrcode is required for mock_qr_stream. Install it with "
                    "'sudo apt install python3-qrcode python3-pil'."
                )
                raise exc

            qr = qrcode.QRCode(version=1, box_size=10, border=4)
            qr.add_data(self.qr_text)
            qr.make(fit=True)
            image = qr.make_image(fill_color="black", back_color="white").convert("L")
            self.cached_image = image.resize((self.image_size_px, self.image_size_px))

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.image_size_px
        msg.width = self.image_size_px
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = self.image_size_px
        msg.data = self.cached_image.tobytes()
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = MockQrStream()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
