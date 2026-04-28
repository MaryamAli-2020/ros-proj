from __future__ import annotations

from functools import partial
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from workflow_interfaces.msg import BinTarget
from workflow_interfaces.srv import ExecuteStage


class CoordinatorNode(Node):
    """Own the high-level workflow state machine for the pick-scan-place demo."""

    def __init__(self) -> None:
        super().__init__("coordinator_node")

        self.auto_start = self.declare_parameter("auto_start", True).value
        self.start_delay_sec = float(self.declare_parameter("start_delay_sec", 6.0).value)
        self.scan_timeout_sec = float(self.declare_parameter("scan_timeout_sec", 10.0).value)
        self.reset_object_before_cycle = self.declare_parameter("reset_object_before_cycle", True).value
        self.perform_initial_home_stage = self.declare_parameter(
            "perform_initial_home_stage", True
        ).value

        self.manipulation_service = self.declare_parameter(
            "manipulation_service", "/manipulation/execute_stage"
        ).value
        self.attachment_service = self.declare_parameter(
            "attachment_service", "/simulation/set_attachment"
        ).value
        self.reset_service = self.declare_parameter("reset_service", "/simulation/reset_object").value
        self.decoded_qr_topic = self.declare_parameter(
            "decoded_qr_topic", "/perception/decoded_qr"
        ).value
        self.bin_target_topic = self.declare_parameter(
            "bin_target_topic", "/decision_logic/bin_target"
        ).value
        self.state_topic = self.declare_parameter("state_topic", "/coordinator/state").value

        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_publisher = self.create_publisher(String, self.state_topic, state_qos)
        self.last_decoded_qr: Optional[str] = None
        self.last_bin_target: Optional[BinTarget] = None
        self.pending_scan_deadline = None
        self.wait_timer = None
        self.auto_start_timer = None
        self.place_stage_requested = False
        self.busy = False
        self.state = "IDLE"

        self.create_subscription(String, self.decoded_qr_topic, self.qr_callback, 10)
        self.create_subscription(BinTarget, self.bin_target_topic, self.bin_target_callback, 10)

        self.execute_stage_client = self.create_client(ExecuteStage, self.manipulation_service)
        self.attachment_client = self.create_client(SetBool, self.attachment_service)
        self.reset_client = self.create_client(Trigger, self.reset_service)

        self.start_service = self.create_service(Trigger, "/coordinator/start_cycle", self.handle_start_cycle)

        self.publish_state("IDLE")
        if self.auto_start:
            self.auto_start_timer = self.create_timer(self.start_delay_sec, self.auto_start_once)

        self.get_logger().info("Coordinator node ready.")

    def auto_start_once(self) -> None:
        if self.busy:
            return
        if self.auto_start_timer is not None:
            self.auto_start_timer.cancel()
            self.auto_start_timer = None
        self.get_logger().info("Auto-start timer elapsed. Beginning workflow cycle.")
        self.start_cycle()

    def handle_start_cycle(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.busy:
            response.success = False
            response.message = "Coordinator is already running a cycle."
            return response

        self.get_logger().info("Received manual start-cycle request.")
        self.start_cycle()
        response.success = True
        response.message = "Cycle started."
        return response

    def qr_callback(self, msg: String) -> None:
        is_duplicate = self.last_decoded_qr == msg.data
        self.last_decoded_qr = msg.data
        if not is_duplicate:
            self.get_logger().info(f"Coordinator observed decoded QR '{self.last_decoded_qr}'.")

    def bin_target_callback(self, msg: BinTarget) -> None:
        is_duplicate = (
            self.last_bin_target is not None
            and self.last_bin_target.bin_id == msg.bin_id
            and self.last_bin_target.qr_text == msg.qr_text
        )
        self.last_bin_target = msg
        if not is_duplicate:
            self.get_logger().info(f"Coordinator received target bin '{msg.bin_id}'.")

        if self.state == "WAIT_FOR_QR":
            self.begin_place_stage("bin target subscription")

    def start_cycle(self) -> None:
        self.busy = True
        self.reset_cycle_context()
        self.get_logger().info("Starting a new pick-scan-place cycle.")

        if self.reset_object_before_cycle:
            self.call_reset(self.after_reset)
        else:
            self.after_reset(True)

    def after_reset(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Failed to reset the simulation object.")
            return
        if self.perform_initial_home_stage:
            self.command_stage("return_home", self.after_home)
        else:
            self.after_home(True)

    def after_home(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Unable to move to the home pose.")
            return
        self.command_stage("move_to_pick", self.after_pre_pick)

    def after_pre_pick(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Unable to move to the pre-grasp pose.")
            return
        self.command_stage("grasp", self.after_pick)

    def after_pick(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Grasp stage failed.")
            return
        self.call_attachment(True, self.after_attach)

    def after_attach(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Simulation attachment could not be enabled.")
            return
        self.command_stage("lift", self.after_lift)

    def after_lift(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Lift motion failed.")
            return
        self.command_stage("move_to_scan", self.after_scan)

    def after_scan(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Scan pose was not reached.")
            return

        self.place_stage_requested = False
        self.publish_state("WAIT_FOR_QR")
        if self.last_bin_target is not None:
            self.begin_place_stage("pre-existing bin target")
            return

        self.pending_scan_deadline = self.get_clock().now().nanoseconds / 1e9 + self.scan_timeout_sec
        self.wait_timer = self.create_timer(0.25, self.poll_for_decision)

    def poll_for_decision(self) -> None:
        if self.last_bin_target is not None:
            self.begin_place_stage("decision poll")
            return

        if self.pending_scan_deadline is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time >= float(self.pending_scan_deadline):
            self.fail_cycle("Timed out waiting for QR-based bin decision.")

    def after_place(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Move-to-place stage failed.")
            return

        self.call_attachment(False, self.after_detach_for_release)

    def after_detach_for_release(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Simulation attachment could not be disabled before release.")
            return
        self.command_stage("release", self.after_release)

    def after_release(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Release stage failed.")
            return
        self.command_stage("return_home", self.finish_cycle)

    def finish_cycle(self, success: bool) -> None:
        if not success:
            self.fail_cycle("Unable to return to home.")
            return

        self.reset_cycle_context()
        self.busy = False
        self.publish_state("IDLE")
        self.get_logger().info("Workflow cycle completed successfully.")

    def fail_cycle(self, reason: str) -> None:
        self.reset_cycle_context()
        self.busy = False
        self.publish_state("ERROR")
        self.get_logger().error(reason)

    def begin_place_stage(self, trigger_source: str) -> None:
        if not self.busy or self.state != "WAIT_FOR_QR":
            self.get_logger().info(
                f"Ignoring place-stage trigger from {trigger_source} while coordinator state is {self.state}."
            )
            return
        if self.place_stage_requested:
            self.get_logger().info(
                f"Ignoring duplicate place-stage trigger from {trigger_source}; stage already requested."
            )
            return
        if self.last_bin_target is None:
            self.get_logger().warn(
                f"Cannot begin place stage from {trigger_source} because no bin target is available."
            )
            return

        self.place_stage_requested = True
        self.cancel_wait_timer()
        self.publish_state("DECIDE_BIN")
        self.get_logger().info(
            f"Advancing to the placement stage using bin '{self.last_bin_target.bin_id}' from {trigger_source}."
        )
        self.command_stage("move_to_place", self.after_place)

    def cancel_wait_timer(self) -> None:
        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None
        self.pending_scan_deadline = None

    def reset_cycle_context(self) -> None:
        self.cancel_wait_timer()
        self.last_decoded_qr = None
        self.last_bin_target = None
        self.place_stage_requested = False

    def command_stage(self, stage: str, next_step: Callable[[bool], None]) -> None:
        self.publish_state(stage.upper())
        if not self.execute_stage_client.wait_for_service(timeout_sec=2.0):
            next_step(False)
            return

        request = ExecuteStage.Request()
        request.stage = stage

        future = self.execute_stage_client.call_async(request)
        future.add_done_callback(partial(self._handle_stage_result, stage, next_step))

    def _handle_stage_result(self, stage: str, next_step: Callable[[bool], None], future) -> None:
        try:
            response = future.result()
            success = bool(response.success)
            if not success:
                self.get_logger().error(f"Stage '{stage}' failed: {response.message}")
            next_step(success)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Stage '{stage}' call failed: {exc}")
            next_step(False)

    def call_attachment(self, enabled: bool, next_step: Callable[[bool], None]) -> None:
        if not self.attachment_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Attachment service is unavailable.")
            next_step(False)
            return

        request = SetBool.Request()
        request.data = enabled
        future = self.attachment_client.call_async(request)
        future.add_done_callback(
            lambda fut: next_step(bool(fut.result().success) if fut.result() else False)
        )

    def call_reset(self, next_step: Callable[[bool], None]) -> None:
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Reset-object service is unavailable.")
            next_step(False)
            return

        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda fut: next_step(bool(fut.result().success) if fut.result() else False)
        )

    def publish_state(self, state: str) -> None:
        self.state = state
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
        self.get_logger().info(f"Coordinator state -> {state}")


def main() -> None:
    rclpy.init()
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
