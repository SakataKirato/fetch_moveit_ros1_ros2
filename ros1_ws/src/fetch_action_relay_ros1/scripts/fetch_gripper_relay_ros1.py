#!/usr/bin/env python3

import json
import math
import threading
import time
import uuid

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)
from std_msgs.msg import String


SUCCESSFUL = 0
INVALID_GOAL = -1


class FetchGripperRelayRos1:
    def __init__(self):
        self.action_name = rospy.get_param(
            "~action_name", "/gripper_controller/gripper_action"
        )
        self.goal_topic = rospy.get_param(
            "~goal_topic", "/fetch_action_relay/gripper_goal"
        )
        self.cancel_topic = rospy.get_param(
            "~cancel_topic", "/fetch_action_relay/gripper_cancel"
        )
        self.result_topic = rospy.get_param(
            "~result_topic", "/fetch_action_relay/gripper_result"
        )
        self.feedback_topic = rospy.get_param(
            "~feedback_topic", "/fetch_action_relay/gripper_feedback"
        )
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.server_wait_timeout = float(
            rospy.get_param("~server_wait_timeout", 5.0)
        )
        self.result_wait_timeout = float(
            rospy.get_param("~result_wait_timeout", 30.0)
        )
        self.queue_size = int(rospy.get_param("~queue_size", 10))

        self._lock = threading.Lock()
        self._active_token = None

        self._result_pub = rospy.Publisher(
            self.result_topic,
            String,
            queue_size=self.queue_size,
            latch=True,
        )
        self._feedback_pub = rospy.Publisher(
            self.feedback_topic, String, queue_size=self.queue_size
        )
        self._goal_sub = rospy.Subscriber(
            self.goal_topic,
            String,
            self._goal_callback,
            queue_size=self.queue_size,
        )
        self._cancel_sub = rospy.Subscriber(
            self.cancel_topic,
            String,
            self._cancel_callback,
            queue_size=self.queue_size,
        )

        self._action_client = actionlib.SimpleActionClient(
            self.action_name, GripperCommandAction
        )
        mode = "DRY RUN" if self.dry_run else "hardware relay"
        rospy.loginfo(
            "fetch_gripper_relay_ros1 ready in %s mode; ROS1 gripper action is %s",
            mode,
            self.action_name,
        )

    def _goal_callback(self, message):
        token = None
        try:
            payload = json.loads(message.data)
            token = str(payload["token"]).strip()
            position = float(payload["position"])
            max_effort = float(payload.get("max_effort", 0.0))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logerr("Rejecting malformed gripper relay goal: %s", exc)
            if token:
                self._publish_result(
                    token,
                    GoalStatus.ABORTED,
                    0.0,
                    0.0,
                    False,
                    False,
                )
            return

        if not token or not math.isfinite(position) or not math.isfinite(max_effort):
            rospy.logerr("Rejecting invalid gripper relay goal %s", token)
            if token:
                self._publish_result(
                    token,
                    GoalStatus.ABORTED,
                    position if math.isfinite(position) else 0.0,
                    0.0,
                    False,
                    False,
                )
            return

        with self._lock:
            if self._active_token is not None:
                rospy.logerr(
                    "Rejecting gripper relay goal %s because another goal is active",
                    token,
                )
                self._publish_result(
                    token,
                    GoalStatus.ABORTED,
                    position,
                    0.0,
                    False,
                    False,
                )
                return
            self._active_token = token

        thread = threading.Thread(
            target=self._dispatch_goal,
            args=(token, position, max_effort),
            name="fetch_gripper_relay_goal",
            daemon=True,
        )
        thread.start()

    def _dispatch_goal(self, token, position, max_effort):
        try:
            rospy.loginfo(
                "Relay gripper goal %s: position=%.6f max_effort=%.6f",
                token,
                position,
                max_effort,
            )
            if self.dry_run:
                self._publish_result(
                    token,
                    GoalStatus.SUCCEEDED,
                    position,
                    0.0,
                    False,
                    True,
                )
                rospy.loginfo(
                    "Dry-run gripper goal %s completed; no ROS1 action goal was sent",
                    token,
                )
                return

            if not self._action_client.wait_for_server(
                rospy.Duration.from_sec(self.server_wait_timeout)
            ):
                rospy.logerr("ROS1 gripper action server is unavailable")
                self._publish_result(
                    token,
                    GoalStatus.ABORTED,
                    position,
                    0.0,
                    False,
                    False,
                )
                return

            goal = GripperCommandGoal()
            goal.command.position = position
            goal.command.max_effort = max_effort
            rospy.loginfo("Sending gripper relay goal %s to %s", token, self.action_name)
            self._action_client.send_goal(
                goal,
                done_cb=lambda status, result: self._done_callback(
                    token, status, result
                ),
                feedback_cb=lambda feedback: self._feedback_callback(token, feedback),
            )
        except Exception as exc:
            rospy.logerr("Gripper relay goal %s failed: %s", token, exc)
            self._publish_result(
                token,
                GoalStatus.ABORTED,
                position,
                0.0,
                False,
                False,
            )
            self._clear_active(token)

    def _cancel_callback(self, message):
        token = message.data.strip()
        with self._lock:
            active_token = self._active_token
        if token and token == active_token:
            rospy.logwarn("Canceling ROS1 gripper goal for relay token %s", token)
            self._action_client.cancel_goal()

    def _feedback_callback(self, token, feedback):
        payload = {
            "token": token,
            "position": float(getattr(feedback, "position", 0.0)),
            "effort": float(getattr(feedback, "effort", 0.0)),
            "stalled": bool(getattr(feedback, "stalled", False)),
            "reached_goal": bool(getattr(feedback, "reached_goal", False)),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._feedback_pub.publish(message)

    def _done_callback(self, token, status, result):
        position = float(getattr(result, "position", 0.0))
        effort = float(getattr(result, "effort", 0.0))
        stalled = bool(getattr(result, "stalled", False))
        reached_goal = bool(getattr(result, "reached_goal", False))
        rospy.loginfo(
            "ROS1 gripper action finished for relay goal %s: "
            "status=%d position=%.6f effort=%.6f stalled=%s reached_goal=%s",
            token,
            status,
            position,
            effort,
            stalled,
            reached_goal,
        )
        self._publish_result(
            token,
            status,
            position,
            effort,
            stalled,
            reached_goal,
        )
        self._clear_active(token)

    def _publish_result(
        self,
        token,
        status,
        position,
        effort,
        stalled,
        reached_goal,
    ):
        self._wait_for_result_subscriber()
        payload = {
            "token": token,
            "status": int(status),
            "position": float(position),
            "effort": float(effort),
            "stalled": bool(stalled),
            "reached_goal": bool(reached_goal),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._result_pub.publish(message)

    def _wait_for_result_subscriber(self):
        deadline = time.monotonic() + max(0.0, self.result_wait_timeout)
        while not rospy.is_shutdown() and self._result_pub.get_num_connections() == 0:
            if time.monotonic() >= deadline:
                rospy.logwarn(
                    "No subscriber connected to %s after %.1f seconds; "
                    "publishing result anyway",
                    self.result_topic,
                    self.result_wait_timeout,
                )
                return
            time.sleep(0.05)

    def _clear_active(self, token):
        with self._lock:
            if self._active_token == token:
                self._active_token = None


def main():
    rospy.init_node("fetch_gripper_relay_ros1")
    FetchGripperRelayRos1()
    rospy.spin()


if __name__ == "__main__":
    main()
