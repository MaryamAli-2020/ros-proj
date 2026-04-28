from __future__ import annotations

import threading
from typing import Iterable, Tuple

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener


def rotate_vector(vector: Iterable[float], quaternion: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    x, y, z = vector
    qx, qy, qz, qw = quaternion

    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z

    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
    return rx, ry, rz


class GraspAttachmentNode(Node):
    """Keep the object aligned with the gripper while the workflow is in a grasped state."""

    def __init__(self) -> None:
        super().__init__("grasp_attachment_node")
        self.callback_group = ReentrantCallbackGroup()

        self.object_name = self.declare_parameter("object_name", "qr_object").value
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.tool_frame = self.declare_parameter("tool_frame", "tool0").value
        self.follow_rate_hz = float(self.declare_parameter("follow_rate_hz", 30.0).value)
        self.attach_offset = list(self.declare_parameter("attach_offset", [-0.05, 0.0, 0.01]).value)
        self.primary_entity_state_service = self.declare_parameter(
            "primary_entity_state_service", "/gazebo/set_entity_state"
        ).value
        self.fallback_entity_state_service = self.declare_parameter(
            "fallback_entity_state_service", "/set_entity_state"
        ).value
        self.reset_pose = list(
            self.declare_parameter("reset_pose", [0.55, 0.0, 0.825, 0.0, 0.0, 0.0, 1.0]).value
        )
        self.service_call_timeout_sec = float(
            self.declare_parameter("service_call_timeout_sec", 0.5).value
        )

        self.is_attached = False
        self.entity_state_call_lock = threading.Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.primary_set_entity_state_client = self.create_client(
            SetEntityState,
            self.primary_entity_state_service,
            callback_group=self.callback_group,
        )
        self.fallback_set_entity_state_client = self.create_client(
            SetEntityState,
            self.fallback_entity_state_service,
            callback_group=self.callback_group,
        )
        self.active_entity_state_service = None

        self.attach_service = self.create_service(
            SetBool,
            "/simulation/set_attachment",
            self.handle_attach,
            callback_group=self.callback_group,
        )
        self.reset_service = self.create_service(
            Trigger,
            "/simulation/reset_object",
            self.handle_reset,
            callback_group=self.callback_group,
        )
        self.timer = self.create_timer(
            1.0 / self.follow_rate_hz,
            self.follow_tool,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Simulation attachment helper ready.")

    def handle_attach(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.is_attached = request.data
        response.success = True
        response.message = "Object attachment enabled." if request.data else "Object attachment disabled."
        self.get_logger().info(response.message)
        return response

    def handle_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.get_logger().info("Reset-object request received.")
        success = self.send_entity_state(self.pose_from_values(self.reset_pose), allow_skip=False)
        response.success = success
        response.message = "Object reset to the pick station." if success else "Failed to reset object pose."
        if success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response

    def follow_tool(self) -> None:
        if not self.is_attached:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.tool_frame, rclpy.time.Time()
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"Waiting for tool transform: {exc}")
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        offset = rotate_vector(
            self.attach_offset,
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

        pose = Pose()
        pose.position.x = translation.x + offset[0]
        pose.position.y = translation.y + offset[1]
        pose.position.z = translation.z + offset[2]
        pose.orientation = rotation

        self.send_entity_state(pose, allow_skip=True)

    def send_entity_state(self, pose: Pose, allow_skip: bool = False) -> bool:
        if allow_skip:
            if not self.entity_state_call_lock.acquire(blocking=False):
                return True
        else:
            self.entity_state_call_lock.acquire()

        try:
            return self._send_entity_state_locked(pose)
        finally:
            self.entity_state_call_lock.release()

    def _send_entity_state_locked(self, pose: Pose) -> bool:
        client = self.get_entity_state_client()
        if client is None:
            return False

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.object_name
        request.state.pose = pose
        request.state.reference_frame = self.world_frame
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0

        if self.call_entity_state_client(client, request):
            return True

        if client == self.primary_set_entity_state_client and self.fallback_set_entity_state_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                f"Retrying the object reset with fallback Gazebo state service '{self.fallback_entity_state_service}'."
            )
            self.active_entity_state_service = self.fallback_entity_state_service
            return self.call_entity_state_client(self.fallback_set_entity_state_client, request)

        return False

    def get_entity_state_client(self):
        if self.primary_set_entity_state_client.wait_for_service(timeout_sec=0.1):
            if self.active_entity_state_service != self.primary_entity_state_service:
                self.active_entity_state_service = self.primary_entity_state_service
                self.get_logger().info(
                    f"Using Gazebo state service '{self.active_entity_state_service}'."
                )
            return self.primary_set_entity_state_client

        if self.fallback_set_entity_state_client.wait_for_service(timeout_sec=0.1):
            if self.active_entity_state_service != self.fallback_entity_state_service:
                self.active_entity_state_service = self.fallback_entity_state_service
                self.get_logger().info(
                    f"Using Gazebo state service '{self.active_entity_state_service}'."
                )
            return self.fallback_set_entity_state_client

        self.get_logger().warn(
            "Waiting for Gazebo entity state service. Checked "
            f"'{self.primary_entity_state_service}' and '{self.fallback_entity_state_service}'."
        )
        return None

    def call_entity_state_client(self, client, request: SetEntityState.Request) -> bool:
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())

        if not completed.wait(timeout=self.service_call_timeout_sec):
            self.get_logger().error(
                f"Timed out waiting for Gazebo state update on '{self.active_entity_state_service}'."
            )
            return False

        result = future.result()
        if result is None:
            self.get_logger().error(
                f"Gazebo state update returned no result on '{self.active_entity_state_service}'."
            )
            return False

        if not result.success:
            self.get_logger().error(
                "Gazebo rejected the entity-state update for "
                f"'{self.object_name}' via '{self.active_entity_state_service}': "
                f"{result.status_message}"
            )
        return bool(result.success)

    def pose_from_values(self, values: list[float]) -> Pose:
        pose = Pose()
        pose.position.x = float(values[0])
        pose.position.y = float(values[1])
        pose.position.z = float(values[2])
        pose.orientation.x = float(values[3])
        pose.orientation.y = float(values[4])
        pose.orientation.z = float(values[5])
        pose.orientation.w = float(values[6])
        return pose


def main() -> None:
    rclpy.init()
    node = GraspAttachmentNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
