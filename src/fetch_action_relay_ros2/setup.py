from setuptools import find_packages, setup


package_name = "fetch_action_relay_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/relay.launch.py", "launch/joy_to_cmd_vel.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="ROS 2 FollowJointTrajectory action server for the ROS 1 Fetch relay",
    license="BSD",
    entry_points={
        "console_scripts": [
            "fetch_action_relay_ros2 = fetch_action_relay_ros2.relay_node:main",
            "fetch_gripper_relay_ros2 = fetch_action_relay_ros2.gripper_relay_node:main",
            "fetch_joy_to_cmd_vel = fetch_action_relay_ros2.joy_to_cmd_vel_node:main",
        ],
    },
)
