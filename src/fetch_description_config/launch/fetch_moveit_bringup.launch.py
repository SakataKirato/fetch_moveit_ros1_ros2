from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, launch_arguments=None, condition=None):
    source = PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare(package_name), "launch", launch_file]
        )
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description=(
                    "Keep the ROS 2 relay local and do not publish goals to ROS 1"
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2",
            ),
            DeclareLaunchArgument(
                "teleop",
                default_value="true",
                description="Start the joystick driver and base teleoperation node",
            ),
            DeclareLaunchArgument(
                "teleop_linear_scale",
                default_value="0.03",
                description="Linear velocity scale for joystick teleoperation",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("teleop")),
            ),
            _include(
                "fetch_action_relay_ros2",
                "joy_to_cmd_vel.launch.py",
                {
                    "linear_scale": LaunchConfiguration("teleop_linear_scale"),
                },
                condition=IfCondition(LaunchConfiguration("teleop")),
            ),
            _include(
                "fetch_action_relay_ros2",
                "relay.launch.py",
                {"dry_run": LaunchConfiguration("dry_run")},
            ),
            _include(
                "fetch_description_config",
                "rsp.launch.py",
            ),
            _include(
                "fetch_description_config",
                "static_virtual_joint_tfs.launch.py",
            ),
            _include(
                "fetch_description_config",
                "move_group.launch.py",
            ),
            _include(
                "fetch_description_config",
                "moveit_rviz.launch.py",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
