#!/usr/bin/env python3

import copy
import json
import threading
import time
import uuid

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


SUCCESSFUL = 0
INVALID_GOAL = -1


class FetchActionRelayRos1:
    def __init__(self):
        self.action_name = rospy.get_param(
            "~action_name", "/arm_controller/follow_joint_trajectory"
        )
        self.goal_topic = rospy.get_param(
            "~goal_topic", "/fetch_action_relay/goal_trajectory"
        )
        self.goal_id_topic = rospy.get_param(
            "~goal_id_topic", "/fetch_action_relay/goal_id"
        )
        self.cancel_topic = rospy.get_param(
            "~cancel_topic", "/fetch_action_relay/cancel"
        )
        self.result_topic = rospy.get_param(
            "~result_topic", "/fetch_action_relay/result"
        )
        self.feedback_topic = rospy.get_param(
            "~feedback_topic", "/fetch_action_relay/feedback"
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
        self._pending_token = None
        self._pending_trajectory = None
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

        self._goal_id_sub = rospy.Subscriber(
            self.goal_id_topic, String, self._goal_id_callback, queue_size=self.queue_size
        )
        self._goal_sub = rospy.Subscriber(
            self.goal_topic,
            JointTrajectory,
            self._trajectory_callback,
            queue_size=self.queue_size,
        )
        self._cancel_sub = rospy.Subscriber(
            self.cancel_topic, String, self._cancel_callback, queue_size=self.queue_size
        )

        self._action_client = actionlib.SimpleActionClient(
            self.action_name, FollowJointTrajectoryAction
        )
        mode = "DRY RUN" if self.dry_run else "hardware relay"
        rospy.loginfo(
            "fetch_action_relay_ros1 ready in %s mode; ROS1 action is %s",
            mode,
            self.action_name,
        )
        rospy.loginfo(
            "Waiting for ROS1 action server is deferred until a trajectory arrives"
        )

    def _goal_id_callback(self, message):
        with self._lock:
            self._pending_token = message.data.strip()
            self._try_start_pending_locked()

    def _trajectory_callback(self, message):
        with self._lock:
            self._pending_trajectory = copy.deepcopy(message)
            self._try_start_pending_locked()

    def _try_start_pending_locked(self):
        if self._pending_token is None or self._pending_trajectory is None:
            return

        token = self._pending_token
        trajectory = self._pending_trajectory
        self._pending_token = None
        self._pending_trajectory = None

        if self._active_token is not None:
            rospy.logerr("Rejecting relay goal %s because another goal is active", token)
            self._publish_result(
                token,
                GoalStatus.ABORTED,
                INVALID_GOAL,
                "ROS1 relay is already executing another goal",
            )
            return

        self._active_token = token
        thread = threading.Thread(
            target=self._dispatch_goal,
            args=(token, trajectory),
            name="fetch_action_relay_goal",
            daemon=True,
        )
        thread.start()

    def _dispatch_goal(self, token, trajectory):
        if not trajectory.joint_names or not trajectory.points:
            rospy.logerr("Rejecting empty trajectory for relay goal %s", token)
            self._publish_result(
                token, GoalStatus.ABORTED, INVALID_GOAL, "Empty trajectory"
            )
            self._clear_active(token)
            return

        self._log_trajectory(token, trajectory)
        if self.dry_run:
            self._publish_result(
                token,
                GoalStatus.SUCCEEDED,
                SUCCESSFUL,
                "Dry-run completed; no ROS1 action goal was sent",
            )
            self._clear_active(token)
            return

        if not self._action_client.wait_for_server(
            rospy.Duration.from_sec(self.server_wait_timeout)
        ):
            error = "ROS1 FollowJointTrajectory action server is unavailable"
            rospy.logerr(error)
            self._publish_result(token, GoalStatus.ABORTED, INVALID_GOAL, error)
            self._clear_active(token)
            return

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        rospy.loginfo(
            "Sending relay goal %s to ROS1 action server (%d joints, %d points)",
            token,
            len(trajectory.joint_names),
            len(trajectory.points),
        )
        self._action_client.send_goal(
            goal,
            done_cb=lambda status, result: self._done_callback(
                token, status, result
            ),
            feedback_cb=lambda feedback: self._feedback_callback(token, feedback),
        )

    def _cancel_callback(self, message):
        token = message.data.strip()
        with self._lock:
            active_token = self._active_token
        if token and token == active_token:
            rospy.logwarn("Canceling ROS1 action goal for relay token %s", token)
            self._action_client.cancel_goal()

    def _feedback_callback(self, token, feedback):
        payload = {
            "token": token,
            "feedback": {
                "header": {
                    "sec": int(feedback.header.stamp.secs),
                    "nanosec": int(feedback.header.stamp.nsecs),
                    "frame_id": feedback.header.frame_id,
                },
                "joint_names": list(feedback.joint_names),
                "desired": self._point_to_dict(feedback.desired),
                "actual": self._point_to_dict(feedback.actual),
                "error": self._point_to_dict(feedback.error),
            },
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._feedback_pub.publish(message)

    def _done_callback(self, token, status, result):
        error_code = int(getattr(result, "error_code", INVALID_GOAL))
        error_string = str(getattr(result, "error_string", ""))
        rospy.loginfo(
            "ROS1 action finished for relay goal %s: status=%d error_code=%d %s",
            token,
            status,
            error_code,
            error_string,
        )
        self._publish_result(token, status, error_code, error_string)
        self._clear_active(token)

    def _publish_result(self, token, status, error_code, error_string):
        self._wait_for_result_subscriber()
        payload = {
            "token": token,
            "status": int(status),
            "error_code": int(error_code),
            "error_string": str(error_string),
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

    @staticmethod
    def _point_to_dict(point):
        return {
            "positions": [float(value) for value in point.positions],
            "velocities": [float(value) for value in point.velocities],
            "accelerations": [float(value) for value in point.accelerations],
            "effort": [float(value) for value in point.effort],
            "time_from_start": {
                "sec": int(point.time_from_start.secs),
                "nanosec": int(point.time_from_start.nsecs),
            },
        }

    @staticmethod
    def _log_trajectory(token, trajectory):
        payload = {
            "header": {
                "sec": int(trajectory.header.stamp.secs),
                "nanosec": int(trajectory.header.stamp.nsecs),
                "frame_id": trajectory.header.frame_id,
            },
            "joint_names": list(trajectory.joint_names),
            "points": [
                {
                    "positions": [float(value) for value in point.positions],
                    "velocities": [float(value) for value in point.velocities],
                    "accelerations": [float(value) for value in point.accelerations],
                    "effort": [float(value) for value in point.effort],
                    "time_from_start": {
                        "sec": int(point.time_from_start.secs),
                        "nanosec": int(point.time_from_start.nsecs),
                    },
                }
                for point in trajectory.points
            ],
        }
        rospy.loginfo(
            "Relay goal %s trajectory: %s",
            token,
            json.dumps(payload, separators=(",", ":")),
        )


def main():
    rospy.init_node("fetch_action_relay_ros1")
    FetchActionRelayRos1()
    rospy.spin()


if __name__ == "__main__":
    main()
