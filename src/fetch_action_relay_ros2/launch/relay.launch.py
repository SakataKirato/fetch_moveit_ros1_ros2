from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="Log goals and return success without sending them to ROS 1",
            ),
            DeclareLaunchArgument(
                "action_name",
                default_value="/arm_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "gripper_action_name",
                default_value="/gripper_controller/gripper_action",
            ),
            Node(
                package="fetch_action_relay_ros2",
                executable="fetch_action_relay_ros2",
                name="fetch_action_relay_ros2",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "action_name": LaunchConfiguration("action_name"),
                    }
                ],
            ),
            Node(
                package="fetch_action_relay_ros2",
                executable="fetch_gripper_relay_ros2",
                name="fetch_gripper_relay_ros2",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "action_name": LaunchConfiguration("gripper_action_name"),
                    }
                ],
            ),
        ]
    )
