cmake_minimum_required(VERSION 3.0.2)
project(fetch_action_relay_ros1)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  rospy
  std_msgs
  trajectory_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/fetch_action_relay_ros1.py
  scripts/fetch_gripper_relay_ros1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/ros1_bridge_dynamic_bridge.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
