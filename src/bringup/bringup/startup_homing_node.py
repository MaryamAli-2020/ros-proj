from __future__ import annotations

from typing import Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class StartupHomingNode(Node):
    """Command the simulated arm into a known home posture before the workflow starts."""

    def __init__(self) -> None:
        super().__init__("startup_homing_node")

        self.arm_action_name = self.declare_parameter(
            "arm_action_name", "/arm_controller/follow_joint_trajectory"
        ).value
        self.gripper_action_name = self.declare_parameter(
            "gripper_action_name", "/gripper_controller/follow_joint_trajectory"
        ).value
        self.arm_joint_names = list(
            self.declare_parameter(
                "arm_joint_names",
                [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_pitch_joint",
                    "wrist_yaw_joint",
                    "wrist_roll_joint",
                ],
            ).value
        )
        self.home_positions = list(
            self.declare_parameter(
                "home_positions",
                [2.40, 0.40, -1.10, 0.75, 1.57, 0.0],
            ).value
        )
        self.gripper_joint_name = self.declare_parameter("gripper_joint_name", "gripper_joint").value
        self.gripper_open_position = float(
            self.declare_parameter("gripper_open_position", 0.04).value
        )
        self.arm_motion_duration = float(self.declare_parameter("arm_motion_duration", 2.5).value)
        self.gripper_motion_duration = float(
            self.declare_parameter("gripper_motion_duration", 1.0).value
        )
        self.trigger_cycle_on_success = self.declare_parameter(
            "trigger_cycle_on_success", True
        ).value
        self.coordinator_start_service = self.declare_parameter(
            "coordinator_start_service", "/coordinator/start_cycle"
        ).value

        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action_name)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, self.gripper_action_name)
        self.coordinator_start_client = self.create_client(
            Trigger, self.coordinator_start_service
        )

    def run(self) -> int:
        self.get_logger().info("Waiting for controller action servers before startup homing.")
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Arm controller action server did not become available.")
            return 1
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Gripper controller action server did not become available.")
            return 1

        if not self.send_goal(
            self.arm_client, self.arm_joint_names, self.home_positions, self.arm_motion_duration
        ):
            self.get_logger().error("Failed to move the arm into the startup home posture.")
            return 1

        if not self.send_goal(
            self.gripper_client,
            [self.gripper_joint_name],
            [self.gripper_open_position],
            self.gripper_motion_duration,
        ):
            self.get_logger().error("Failed to open the gripper during startup homing.")
            return 1

        if self.trigger_cycle_on_success and not self.start_workflow_cycle():
            self.get_logger().error("Startup homing succeeded, but the coordinator cycle could not be started.")
            return 1

        self.get_logger().info("Startup homing completed successfully.")
        return 0

    def send_goal(
        self,
        client: ActionClient,
        joint_names: Sequence[str],
        positions: Sequence[float],
        duration_sec: float,
    ) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(float(position) for position in positions)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points = [point]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)
        result = result_future.result()
        if result is None:
            return False
        return int(result.result.error_code) == 0

    def start_workflow_cycle(self) -> bool:
        self.get_logger().info("Waiting for the coordinator start service.")
        if not self.coordinator_start_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("Coordinator start service did not become available.")
            return False

        future = self.coordinator_start_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            self.get_logger().error("Coordinator start request returned no response.")
            return False

        if not response.success:
            self.get_logger().error(f"Coordinator rejected startup cycle request: {response.message}")
            return False

        self.get_logger().info(f"Coordinator cycle started automatically: {response.message}")
        return True


def main() -> None:
    rclpy.init()
    node = StartupHomingNode()
    exit_code = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
