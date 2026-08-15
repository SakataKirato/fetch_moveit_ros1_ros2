# Fetch MoveIt ROS 1 / ROS 2

This repository connects a ROS 1 Fetch robot to MoveIt 2 running on ROS 2
Humble.

```text
Fetch (ROS 1 Noetic)
        |
        | ros1_bridge + action relay
        |
MoveIt 2 / RViz (ROS 2 Humble)
```

The ROS 1 environment runs in Docker. The ROS 2 environment used by MoveIt 2
runs on the host. A ROS 2 Humble workspace is also built inside the container
because `ros1_bridge` requires both ROS 1 and ROS 2 libraries.

## Requirements

- Ubuntu 22.04 host
- Docker
- ROS 2 Humble on the host
- Fetch robot

## Docker setup

The Docker configuration is included in this repository under `docker/`.
From the repository root:

```bash
make noetic.build
make noetic.create
make noetic.shell
```

Use `make noetic.up` when the container already exists but is stopped.

Runtime network settings are handled by `docker/scripts/run.bash`. First check
the Fetch IP on the robot:

```bash
# On Fetch
ip a
```

Then set `ROS_MASTER_URI` to the Fetch IP and `ROS_IP` to the bridge PC IP:

```bash
# On the bridge PC
export ROS_MASTER_URI=http://<fetch-ip>:11311
ip a
export ROS_IP=<bridge-pc-ip>
make noetic.create
```

## First-time workspace setup

Run the following inside the container. The ROS 2 Humble and
`ros1_bridge_ws` directories are mounted from the host and are not stored in
Git.

Before building ROS 2 Humble, clear the ROS 1 environment:

```bash
unset ROS_VERSION ROS_DISTRO ROS_PYTHON_VERSION
unset ROS_ROOT ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
```

Install ROS 2 Humble from source by following the
[official Ubuntu development setup](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).

Build `ros1_bridge`:

```bash
mkdir -p ~/ros1_bridge_ws/src
git clone https://github.com/ros2/ros1_bridge.git \
  ~/ros1_bridge_ws/src/ros1_bridge

sed -i \
's/ros::AsyncSpinner async_spinner(1);/ros::AsyncSpinner async_spinner(4);/' \
~/ros1_bridge_ws/src/ros1_bridge/src/dynamic_bridge.cpp

unset PYTHONHOME PYTHONPATH
source /opt/ros/noetic/setup.bash
source ~/ros2_humble/install/setup.bash

cd ~/ros1_bridge_ws
colcon build \
  --packages-select ros1_bridge \
  --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=OFF
```

Build the ROS 1 workspace:

```bash
cd /workspace
source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic
catkin build
source devel/setup.bash
```

## Host-side ROS 2 workspace

Build the ROS 2 packages on the host:

```bash
cd ~/fetch_moveit_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src
source install/setup.bash
```

Source both files again in every new host terminal before using MoveIt:

```bash
source /opt/ros/humble/setup.bash
source ~/fetch_moveit_ws/install/setup.bash
```

## Run and test

Inside the container, start the ROS 1 relay and bridge in dry-run mode:

```bash
roslaunch fetch_action_relay_ros1 ros1_bringup.launch dry_run:=true
```

On the host, start MoveIt 2:

```bash
source /opt/ros/humble/setup.bash
source ~/fetch_moveit_ws/install/setup.bash
ros2 launch fetch_description_config fetch_moveit_bringup.launch.py
```
For real hardware execution, stop the dry-run relay and restart it with:

```bash
roslaunch fetch_action_relay_ros1 ros1_bringup.launch dry_run:=false
```
