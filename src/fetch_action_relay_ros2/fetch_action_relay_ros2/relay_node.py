#!/usr/bin/env python3

import json
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


SUCCESSFUL = 0
INVALID_GOAL = -1
INVALID_JOINTS = -2
OLD_HEADER_TIMESTAMP = -3
PATH_TOLERANCE_VIOLATED = -4
GOAL_TOLERANCE_VIOLATED = -5

# actionlib_msgs/GoalStatus values used by the ROS 1 relay.
STATUS_PREEMPTED = 2
STATUS_SUCCEEDED = 3
STATUS_ABORTED = 4
STATUS_REJECTED = 5
STATUS_RECALLED = 8


@dataclass
class RelayContext:
    token: str
    goal_handle: Any
    done: threading.Event = field(default_factory=threading.Event)
    result_code: int = INVALID_GOAL
    result_string: str = ""
    status: int = STATUS_ABORTED
    result_received: bool = False
    cancel_sent: bool = False


def _duration_to_dict(duration: DurationMsg) -> Dict[str, int]:
    return {"sec": int(duration.sec), "nanosec": int(duration.nanosec)}


def _point_to_dict(point: JointTrajectoryPoint) -> Dict[str, Any]:
    return {
        "positions": [float(value) for value in point.positions],
        "velocities": [float(value) for value in point.velocities],
        "accelerations": [float(value) for value in point.accelerations],
        "effort": [float(value) for value in point.effort],
        "time_from_start": _duration_to_dict(point.time_from_start),
    }


def _trajectory_to_dict(trajectory: JointTrajectory) -> Dict[str, Any]:
    return {
        "header": {
            "sec": int(trajectory.header.stamp.sec),
            "nanosec": int(trajectory.header.stamp.nanosec),
            "frame_id": trajectory.header.frame_id,
        },
        "joint_names": list(trajectory.joint_names),
        "points": [_point_to_dict(point) for point in trajectory.points],
    }


def _set_duration(duration: DurationMsg, value: Dict[str, Any]) -> None:
    duration.sec = int(value.get("sec", 0))
    duration.nanosec = int(value.get("nanosec", 0))


def _dict_to_point(value: Dict[str, Any]) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.positions = [float(item) for item in value.get("positions", [])]
    point.velocities = [float(item) for item in value.get("velocities", [])]
    point.accelerations = [float(item) for item in value.get("accelerations", [])]
    point.effort = [float(item) for item in value.get("effort", [])]
    _set_duration(point.time_from_start, value.get("time_from_start", {}))
    return point


class FetchActionRelayRos2(Node):
    """Expose the MoveIt-facing ROS 2 action and relay it through ROS 1 topics."""

    def __init__(self) -> None:
        super().__init__("fetch_action_relay_ros2")

        self.declare_parameter(
            "action_name", "/arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter(
            "goal_topic", "/fetch_action_relay/goal_trajectory"
        )
        self.declare_parameter("goal_id_topic", "/fetch_action_relay/goal_id")
        self.declare_parameter("cancel_topic", "/fetch_action_relay/cancel")
        self.declare_parameter("result_topic", "/fetch_action_relay/result")
        self.declare_parameter("feedback_topic", "/fetch_action_relay/feedback")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("dry_run_delay_sec", 0.1)
        self.declare_parameter("result_timeout_sec", 120.0)
        self.declare_parameter("cancel_timeout_sec", 5.0)

        self._action_name = str(self.get_parameter("action_name").value)
        self._goal_topic = str(self.get_parameter("goal_topic").value)
        self._goal_id_topic = str(self.get_parameter("goal_id_topic").value)
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

        self._goal_id_pub = self.create_publisher(String, self._goal_id_topic, qos)
        self._goal_pub = self.create_publisher(JointTrajectory, self._goal_topic, qos)
        self._cancel_pub = self.create_publisher(String, self._cancel_topic, qos)

        self._result_sub = self.create_subscription(
            String, self._result_topic, self._result_callback, qos
        )
        self._feedback_sub = self.create_subscription(
            String, self._feedback_topic, self._feedback_callback, qos
        )

        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active: Optional[RelayContext] = None
        self._goal_reserved = False

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self._action_name,
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        mode = "DRY RUN" if self._dry_run else "hardware relay"
        self.get_logger().info(
            f"Serving {self._action_name} in {mode} mode; "
            f"ROS1 goal topic is {self._goal_topic}"
        )

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> int:
        trajectory = goal_request.trajectory
        if not trajectory.joint_names or not trajectory.points:
            self.get_logger().warning("Rejecting an empty FollowJointTrajectory goal")
            return GoalResponse.REJECT

        if goal_request.multi_dof_trajectory.points:
            self.get_logger().warning(
                "Rejecting a goal with multi_dof_trajectory; arm-only relay supports "
                "trajectory_msgs/JointTrajectory"
            )
            return GoalResponse.REJECT

        with self._lock:
            if self._active is not None or self._goal_reserved:
                self.get_logger().warning("Rejecting a goal while another goal is active")
                return GoalResponse.REJECT
            self._goal_reserved = True

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: Any) -> int:
        with self._lock:
            context = self._active
            if context is not None and context.goal_handle is goal_handle:
                self._send_cancel_locked(context)
                self.get_logger().info(f"Cancel requested for relay goal {context.token}")
        return CancelResponse.ACCEPT

    def _send_cancel_locked(self, context: RelayContext) -> None:
        if context.cancel_sent or self._dry_run:
            return
        context.cancel_sent = True
        message = String()
        message.data = context.token
        self._cancel_pub.publish(message)

    def _execute_callback(self, goal_handle: Any) -> FollowJointTrajectory.Result:
        context = RelayContext(token=uuid.uuid4().hex, goal_handle=goal_handle)
        with self._lock:
            self._active = context

        try:
            trajectory = goal_handle.request.trajectory
            if self._dry_run:
                self._log_dry_run(context.token, trajectory)
                if not self._wait_for_dry_run(context):
                    result = FollowJointTrajectory.Result()
                    result.error_code = INVALID_GOAL
                    result.error_string = "Dry-run goal canceled"
                    goal_handle.canceled()
                    return result

                result = FollowJointTrajectory.Result()
                result.error_code = SUCCESSFUL
                result.error_string = "Dry-run completed; no ROS1 goal was sent"
                goal_handle.succeed()
                return result

            goal_id = String()
            goal_id.data = context.token
            self._goal_id_pub.publish(goal_id)
            self._goal_pub.publish(trajectory)
            self.get_logger().info(
                f"Forwarded relay goal {context.token} with "
                f"{len(trajectory.joint_names)} joints and {len(trajectory.points)} points"
            )

            deadline = time.monotonic() + self._result_timeout_sec
            while not context.done.wait(timeout=0.05):
                if goal_handle.is_cancel_requested:
                    with self._lock:
                        self._send_cancel_locked(context)
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        f"Timed out waiting for ROS1 result for relay goal {context.token}"
                    )
                    with self._lock:
                        self._send_cancel_locked(context)
                    context.done.wait(timeout=self._cancel_timeout_sec)
                    if not context.result_received:
                        context.result_code = INVALID_GOAL
                        context.result_string = (
                            "Timed out waiting for the ROS1 FollowJointTrajectory result"
                        )
                        context.status = STATUS_ABORTED
                        break

            result = FollowJointTrajectory.Result()
            result.error_code = int(context.result_code)
            result.error_string = context.result_string

            if goal_handle.is_cancel_requested or context.status in (
                STATUS_PREEMPTED,
                STATUS_RECALLED,
            ):
                goal_handle.canceled()
            elif (
                context.status == STATUS_SUCCEEDED
                and context.result_code == SUCCESSFUL
            ):
                goal_handle.succeed()
            else:
                goal_handle.abort()

            return result
        except Exception as exc:  # keep the action protocol well-formed on relay errors
            self.get_logger().error(f"Relay execution failed: {exc}")
            result = FollowJointTrajectory.Result()
            result.error_code = INVALID_GOAL
            result.error_string = f"ROS2 relay exception: {exc}"
            goal_handle.abort()
            return result
        finally:
            with self._lock:
                if self._active is context:
                    self._active = None
                self._goal_reserved = False

    def _wait_for_dry_run(self, context: RelayContext) -> bool:
        deadline = time.monotonic() + max(0.0, self._dry_run_delay_sec)
        while time.monotonic() < deadline:
            if context.goal_handle.is_cancel_requested:
                return False
            time.sleep(0.02)
        return not context.goal_handle.is_cancel_requested

    def _log_dry_run(self, token: str, trajectory: JointTrajectory) -> None:
        payload = _trajectory_to_dict(trajectory)
        self.get_logger().info(
            f"DRY RUN relay goal {token}: {json.dumps(payload, separators=(',', ':'))}"
        )

    def _result_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            token = str(payload["token"])
            result_code = int(payload.get("error_code", INVALID_GOAL))
            status = int(payload.get("status", STATUS_ABORTED))
            result_string = str(payload.get("error_string", ""))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Ignoring malformed relay result: {exc}")
            return

        with self._lock:
            context = self._active
            if context is None or context.token != token:
                self.get_logger().warning(
                    f"Ignoring result for unknown relay goal {token}"
                )
                return
            context.result_code = result_code
            context.result_string = result_string
            context.status = status
            context.result_received = True
            context.done.set()

    def _feedback_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            token = str(payload["token"])
            feedback_payload = payload["feedback"]
        except (KeyError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Ignoring malformed relay feedback: {exc}")
            return

        with self._lock:
            context = self._active
            if context is None or context.token != token:
                return
            goal_handle = context.goal_handle

        feedback = FollowJointTrajectory.Feedback()
        header = feedback_payload.get("header", {})
        feedback.header.stamp.sec = int(header.get("sec", 0))
        feedback.header.stamp.nanosec = int(header.get("nanosec", 0))
        feedback.header.frame_id = str(header.get("frame_id", ""))
        feedback.joint_names = [str(name) for name in feedback_payload.get("joint_names", [])]
        feedback.desired = _dict_to_point(feedback_payload.get("desired", {}))
        feedback.actual = _dict_to_point(feedback_payload.get("actual", {}))
        feedback.error = _dict_to_point(feedback_payload.get("error", {}))
        goal_handle.publish_feedback(feedback)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FetchActionRelayRos2()
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
