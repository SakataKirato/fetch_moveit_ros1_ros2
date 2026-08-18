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
            DeclareLaunchArgument(
                "head_action_name",
                default_value="/head_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "torso_action_name",
                default_value="/torso_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "arm_with_torso_action_name",
                default_value="/arm_with_torso_controller/follow_joint_trajectory",
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
            Node(
                package="fetch_action_relay_ros2",
                executable="fetch_action_relay_ros2",
                name="fetch_head_action_relay_ros2",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "action_name": LaunchConfiguration("head_action_name"),
                        "goal_topic": "/fetch_action_relay/head_goal_trajectory",
                        "goal_id_topic": "/fetch_action_relay/head_goal_id",
                        "cancel_topic": "/fetch_action_relay/head_cancel",
                        "result_topic": "/fetch_action_relay/head_result",
                        "feedback_topic": "/fetch_action_relay/head_feedback",
                    }
                ],
            ),
            Node(
                package="fetch_action_relay_ros2",
                executable="fetch_action_relay_ros2",
                name="fetch_torso_action_relay_ros2",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "action_name": LaunchConfiguration("torso_action_name"),
                        "goal_topic": "/fetch_action_relay/torso_goal_trajectory",
                        "goal_id_topic": "/fetch_action_relay/torso_goal_id",
                        "cancel_topic": "/fetch_action_relay/torso_cancel",
                        "result_topic": "/fetch_action_relay/torso_result",
                        "feedback_topic": "/fetch_action_relay/torso_feedback",
                    }
                ],
            ),
            Node(
                package="fetch_action_relay_ros2",
                executable="fetch_action_relay_ros2",
                name="fetch_arm_with_torso_action_relay_ros2",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "action_name": LaunchConfiguration(
                            "arm_with_torso_action_name"
                        ),
                        "goal_topic": (
                            "/fetch_action_relay/arm_with_torso_goal_trajectory"
                        ),
                        "goal_id_topic": "/fetch_action_relay/arm_with_torso_goal_id",
                        "cancel_topic": "/fetch_action_relay/arm_with_torso_cancel",
                        "result_topic": "/fetch_action_relay/arm_with_torso_result",
                        "feedback_topic": "/fetch_action_relay/arm_with_torso_feedback",
                    }
                ],
            ),
        ]
    )
