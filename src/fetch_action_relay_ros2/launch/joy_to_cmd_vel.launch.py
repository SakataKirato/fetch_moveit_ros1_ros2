from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("linear_scale", default_value="0.1"),
            DeclareLaunchArgument("joy_timeout_sec", default_value="0.5"),
            Node(
                package="fetch_action_relay_ros2",
                executable="fetch_joy_to_cmd_vel",
                name="fetch_joy_to_cmd_vel",
                output="screen",
                parameters=[
                    {
                        "joy_topic": LaunchConfiguration("joy_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "linear_scale": ParameterValue(
                            LaunchConfiguration("linear_scale"), value_type=float
                        ),
                        "joy_timeout_sec": ParameterValue(
                            LaunchConfiguration("joy_timeout_sec"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
