#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyToCmdVelNode(Node):
    """Convert the requested joystick layout into a base Twist command."""

    def __init__(self) -> None:
        super().__init__("fetch_joy_to_cmd_vel")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_scale", 0.1)
        self.declare_parameter("joy_timeout_sec", 0.5)

        joy_topic = str(self.get_parameter("joy_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._linear_scale = float(self.get_parameter("linear_scale").value)
        self._joy_timeout_sec = float(
            self.get_parameter("joy_timeout_sec").value
        )

        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._joy_sub = self.create_subscription(
            Joy, joy_topic, self._joy_callback, 10
        )

        self._last_joy_time = self.get_clock().now()
        self._last_command_was_nonzero = False
        timer_period = max(0.05, min(0.1, self._joy_timeout_sec / 2.0))
        self._timeout_timer = self.create_timer(
            timer_period, self._publish_stop_on_timeout
        )

        self.get_logger().info(
            f"Converting {joy_topic} to {cmd_vel_topic}; "
            "axes[0]=linear.x; linear.y and angular.z are disabled"
        )

    @staticmethod
    def _axis(axes, index: int) -> float:
        if index >= len(axes):
            return 0.0
        value = float(axes[index])
        if not math.isfinite(value):
            return 0.0
        return max(-1.0, min(1.0, value))

    def _joy_callback(self, joy: Joy) -> None:
        command = Twist()
        command.linear.x = self._axis(joy.axes, 0) * self._linear_scale

        self._cmd_vel_pub.publish(command)
        self._last_joy_time = self.get_clock().now()
        self._last_command_was_nonzero = any(
            (
                command.linear.x,
                command.linear.y,
                command.linear.z,
                command.angular.x,
                command.angular.y,
                command.angular.z,
            )
        )

    def _publish_stop_on_timeout(self) -> None:
        elapsed_sec = (
            self.get_clock().now() - self._last_joy_time
        ).nanoseconds / 1e9
        if elapsed_sec <= self._joy_timeout_sec:
            return

        if self._last_command_was_nonzero:
            self._cmd_vel_pub.publish(Twist())
            self._last_command_was_nonzero = False
            self.get_logger().warning(
                "Joystick messages timed out; published a zero Twist"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyToCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
