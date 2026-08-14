#!/usr/bin/env python3

import json
import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Optional

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


STATUS_PREEMPTED = 2
STATUS_SUCCEEDED = 3
STATUS_ABORTED = 4
STATUS_REJECTED = 5
STATUS_RECALLED = 8


@dataclass
class GripperRelayContext:
    token: str
    goal_handle: Any
    done: threading.Event = field(default_factory=threading.Event)
    status: int = STATUS_ABORTED
    position: float = 0.0
    effort: float = 0.0
    stalled: bool = False
    reached_goal: bool = False
    result_received: bool = False
    cancel_sent: bool = False


class FetchGripperRelayRos2(Node):
    """Expose Fetch's ROS 1 GripperCommand action to MoveIt 2."""

    def __init__(self) -> None:
        super().__init__("fetch_gripper_relay_ros2")

        self.declare_parameter(
            "action_name", "/gripper_controller/gripper_action"
        )
        self.declare_parameter(
            "goal_topic", "/fetch_action_relay/gripper_goal"
        )
        self.declare_parameter(
            "cancel_topic", "/fetch_action_relay/gripper_cancel"
        )
        self.declare_parameter(
            "result_topic", "/fetch_action_relay/gripper_result"
        )
        self.declare_parameter(
            "feedback_topic", "/fetch_action_relay/gripper_feedback"
        )
        self.declare_parameter("dry_run", False)
        self.declare_parameter("dry_run_delay_sec", 0.1)
        self.declare_parameter("result_timeout_sec", 120.0)
        self.declare_parameter("cancel_timeout_sec", 5.0)

        self._action_name = str(self.get_parameter("action_name").value)
        self._goal_topic = str(self.get_parameter("goal_topic").value)
        self._cancel_topic = str(self.get_parameter("cancel_topic").value)
        self._result_topic = str(self.get_parameter("result_topic").value)
        self._feedback_topic = str(self.get_parameter("feedback_topic").value)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._dry_run_delay_sec = float(
            self.get_parameter("dry_run_delay_sec").value
        )
        self._result_timeout_sec = float(
            self.get_parameter("result_timeout_sec").value
        )
        self._cancel_timeout_sec = float(
            self.get_parameter("cancel_timeout_sec").value
        )

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self._goal_pub = self.create_publisher(String, self._goal_topic, qos)
        self._cancel_pub = self.create_publisher(String, self._cancel_topic, qos)
        self._result_sub = self.create_subscription(
            String, self._result_topic, self._result_callback, qos
        )
        self._feedback_sub = self.create_subscription(
            String, self._feedback_topic, self._feedback_callback, qos
        )

        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active: Optional[GripperRelayContext] = None
        self._goal_reserved = False

        self._action_server = ActionServer(
            self,
            GripperCommand,
            self._action_name,
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        mode = "DRY RUN" if self._dry_run else "hardware relay"
        self.get_logger().info(
            f"Serving {self._action_name} in {mode} mode; "
            f"ROS1 gripper goal topic is {self._goal_topic}"
        )

    def _goal_callback(self, goal_request: GripperCommand.Goal) -> int:
        command = goal_request.command
        if not math.isfinite(command.position) or not math.isfinite(
            command.max_effort
        ):
            self.get_logger().warning("Rejecting a gripper goal containing NaN/Inf")
            return GoalResponse.REJECT

        with self._lock:
            if self._active is not None or self._goal_reserved:
                self.get_logger().warning(
                    "Rejecting a gripper goal while another goal is active"
                )
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: Any) -> int:
        with self._lock:
            context = self._active
            if context is not None and context.goal_handle is goal_handle:
                self._send_cancel_locked(context)
                self.get_logger().info(
                    f"Cancel requested for gripper relay goal {context.token}"
                )
        return CancelResponse.ACCEPT

    def _send_cancel_locked(self, context: GripperRelayContext) -> None:
        if context.cancel_sent or self._dry_run:
            return
        context.cancel_sent = True
        message = String()
        message.data = context.token
        self._cancel_pub.publish(message)

    def _execute_callback(self, goal_handle: Any) -> GripperCommand.Result:
        context = GripperRelayContext(
            token=uuid.uuid4().hex,
            goal_handle=goal_handle,
        )
        with self._lock:
            self._active = context

        command = goal_handle.request.command
        try:
            if self._dry_run:
                self._log_dry_run(context.token, command)
                if not self._wait_for_dry_run(context):
                    result = self._make_result(
                        command.position,
                        0.0,
                        False,
                        False,
                    )
                    goal_handle.canceled()
                    return result

                result = self._make_result(
                    command.position,
                    0.0,
                    False,
                    True,
                )
                goal_handle.succeed()
                return result

            goal = {
                "token": context.token,
                "position": float(command.position),
                "max_effort": float(command.max_effort),
            }
            message = String()
            message.data = json.dumps(goal, separators=(",", ":"))
            self._goal_pub.publish(message)
            self.get_logger().info(
                f"Forwarded gripper relay goal {context.token}: "
                f"position={command.position:.6f}, max_effort={command.max_effort:.6f}"
            )

            deadline = time.monotonic() + self._result_timeout_sec
            while not context.done.wait(timeout=0.05):
                if goal_handle.is_cancel_requested:
                    with self._lock:
                        self._send_cancel_locked(context)
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Timed out waiting for ROS1 gripper result "
                        f"for relay goal {context.token}"
                    )
                    with self._lock:
                        self._send_cancel_locked(context)
                    context.done.wait(timeout=self._cancel_timeout_sec)
                    if not context.result_received:
                        context.status = STATUS_ABORTED
                        break

            result = self._make_result(
                context.position,
                context.effort,
                context.stalled,
                context.reached_goal,
            )
            if goal_handle.is_cancel_requested or context.status in (
                STATUS_PREEMPTED,
                STATUS_RECALLED,
            ):
                goal_handle.canceled()
            elif context.status == STATUS_SUCCEEDED:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:  # keep the action protocol well-formed
            self.get_logger().error(f"Gripper relay execution failed: {exc}")
            goal_handle.abort()
            return self._make_result(
                command.position,
                0.0,
                False,
                False,
            )
        finally:
            with self._lock:
                if self._active is context:
                    self._active = None
                self._goal_reserved = False

    def _wait_for_dry_run(self, context: GripperRelayContext) -> bool:
        deadline = time.monotonic() + max(0.0, self._dry_run_delay_sec)
        while time.monotonic() < deadline:
            if context.goal_handle.is_cancel_requested:
                return False
            time.sleep(0.02)
        return not context.goal_handle.is_cancel_requested

    def _result_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            token = str(payload["token"])
            status = int(payload.get("status", STATUS_ABORTED))
            position = float(payload.get("position", 0.0))
            effort = float(payload.get("effort", 0.0))
            stalled = bool(payload.get("stalled", False))
            reached_goal = bool(payload.get("reached_goal", False))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Ignoring malformed gripper result: {exc}")
            return

        with self._lock:
            context = self._active
            if context is None or context.token != token:
                self.get_logger().warning(
                    f"Ignoring result for unknown gripper relay goal {token}"
                )
                return
            context.status = status
            context.position = position
            context.effort = effort
            context.stalled = stalled
            context.reached_goal = reached_goal
            context.result_received = True
            context.done.set()

    def _feedback_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            token = str(payload["token"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Ignoring malformed gripper feedback: {exc}")
            return

        with self._lock:
            context = self._active
            if context is None or context.token != token:
                return
            goal_handle = context.goal_handle

        feedback = GripperCommand.Feedback()
        feedback.position = float(payload.get("position", 0.0))
        feedback.effort = float(payload.get("effort", 0.0))
        feedback.stalled = bool(payload.get("stalled", False))
        feedback.reached_goal = bool(payload.get("reached_goal", False))
        goal_handle.publish_feedback(feedback)

    def _log_dry_run(self, token: str, command: Any) -> None:
        payload = {
            "token": token,
            "position": float(command.position),
            "max_effort": float(command.max_effort),
        }
        self.get_logger().info(
            f"DRY RUN gripper goal: {json.dumps(payload, separators=(',', ':'))}"
        )

    @staticmethod
    def _make_result(
        position: float,
        effort: float,
        stalled: bool,
        reached_goal: bool,
    ) -> GripperCommand.Result:
        result = GripperCommand.Result()
        result.position = float(position)
        result.effort = float(effort)
        result.stalled = bool(stalled)
        result.reached_goal = bool(reached_goal)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FetchGripperRelayRos2()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
