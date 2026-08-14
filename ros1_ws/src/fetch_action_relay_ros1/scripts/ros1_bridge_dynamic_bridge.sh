#!/usr/bin/env bash

set -Eeuo pipefail

# roslaunch starts this wrapper from a ROS 1 environment.  The bridge itself
# is a ROS 2 executable, so load its combined bridge workspace before exec'ing
# it.  ROS_MASTER_URI and ROS_IP are intentionally inherited from roslaunch.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
set +u
source /home/ros/ros1_bridge_ws/install/setup.bash
set -u

exec /home/ros/ros1_bridge_ws/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge "$@"
