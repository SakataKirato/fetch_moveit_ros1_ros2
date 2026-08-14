# Fetch ROS1/ROS2 FollowJointTrajectory relay

This workspace contains two isolated relay nodes:

```text
MoveIt 2 -- FollowJointTrajectory action --> fetch_action_relay_ros2
                                              |
                                              | ros1_bridge standard topics
                                              v
                                      fetch_action_relay_ros1
                                              |
                                              v
Fetch ROS1 /arm_controller/follow_joint_trajectory
```

The trajectory itself uses `trajectory_msgs/JointTrajectory`. Goal IDs,
cancel requests, results, and feedback use `std_msgs/String` containing JSON,
so no custom message or `ros1_bridge` rebuild is required. The ROS 1 relay
copies `positions`, `velocities`, `accelerations`, `effort`, and
`time_from_start` without changing them.

## Important bridge direction

The current bridge command is ROS 1 to ROS 2 only:

```text
--bridge-all-1to2-topics
```

The relay goal travels in the other direction. Keep the existing bridge for
ROS 1 state, and start a second, directional bridge for ROS 2 to ROS 1 topics:

```bash
docker exec -it ros-noetic-a bash -lc '
  source /home/ros/ros1_bridge_ws/install/setup.bash
  export ROS_MASTER_URI=http://192.168.50.130:11311
  export ROS_IP=192.168.50.59
  /home/ros/ros1_bridge_ws/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge \
    --bridge-all-2to1-topics
'
```

Alternatively, replace the existing bridge option with
`--bridge-all-topics`; rebuilding `ros1_bridge` is not needed.

## Build

ROS 2 host:

```bash
cd /home/a/fetch_moveit_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select fetch_description_config fetch_action_relay_ros2
source install/setup.bash
```

ROS 1 container:

```bash
docker exec -it ros-noetic-a bash -lc '
  cd /workspace
  unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
  source /opt/ros/noetic/setup.bash
  catkin config \
    --install \
    --extend /opt/ros/noetic \
    --build-space /workspace/catkin_build \
    --devel-space /workspace/catkin_devel \
    --install-space /workspace/catkin_install \
    --log-space /workspace/catkin_log
  catkin build fetch_action_relay_ros1
'
```

The ROS 1 package is kept under `ros1_ws/src` in this repository. Copy it to
the container-mounted ROS 1 workspace before building if it is not already
present at `/home/a/ros-workspace/src`:

```bash
mkdir -p /home/a/ros-workspace/src
cp -a /home/a/fetch_moveit_ws/ros1_ws/src/fetch_action_relay_ros1 \
  /home/a/ros-workspace/src/
```

## Start

For the simple two-terminal workflow, use the two parent launch files. Each
parent launch reuses the existing child launch files, so the individual
components do not need to be started manually.

In the Docker ROS 1 terminal:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_install/setup.bash
export ROS_MASTER_URI=http://192.168.50.130:11311
export ROS_IP=192.168.50.59
roslaunch fetch_action_relay_ros1 ros1_bringup.launch dry_run:=true
```

On the host ROS 2 terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/a/fetch_moveit_ws/install/setup.bash
ros2 launch fetch_description_config fetch_moveit_bringup.launch.py
```

The Docker launch starts both ROS 1 relay nodes and `dynamic_bridge`. The host
launch starts the ROS 2 arm/gripper relay, robot state publisher, static TF,
MoveIt, and RViz.

For real hardware, change only the Docker-side argument after the dry-run
check:

```bash
roslaunch fetch_action_relay_ros1 ros1_bringup.launch dry_run:=false
```

`dry_run:=true` is the recommended first run: the full ROS 2-to-ROS 1 relay
path is exercised, but the ROS 1 relay does not send the trajectory to Fetch.
For real hardware, change the ROS 1 launch argument to `dry_run:=false` after
the dry-run check. The following commands are the normal two-terminal
workflow.

Start the ROS 1 relay in the container, after the Fetch ROS master and the
2-to-1 bridge are available:

```bash
docker exec -it ros-noetic-a bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/catkin_install/setup.bash
  export ROS_MASTER_URI=http://192.168.50.130:11311
  export ROS_IP=192.168.50.59
  roslaunch fetch_action_relay_ros1 ros1_bringup.launch
'
```

Start the ROS 2 relay in a separate host terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/a/fetch_moveit_ws/install/setup.bash
ros2 launch fetch_action_relay_ros2 relay.launch.py
```

Do not start the generated fake `ros2_control` arm controller at the same time
as this relay, because both would claim the same ROS 2 action name. Use the
MoveIt launch that does not spawn the fake controller for real hardware.

## Dry run

To verify the ROS 2 action server without sending anything to ROS 1:

```bash
ros2 launch fetch_action_relay_ros2 relay.launch.py dry_run:=true
ros2 action info /arm_controller/follow_joint_trajectory
```

For an end-to-end bridge test that logs the trajectory in ROS 1 but does not
call the Fetch action server, use `dry_run:=true` on the ROS 1 launch and keep
the ROS 2 relay in normal mode.

## Verification

The expected action graph is:

```bash
ros2 action info /arm_controller/follow_joint_trajectory
# Action servers: 1
```

Before any hardware test, verify the ROS 1 action without sending a goal:

```bash
docker exec -it ros-noetic-a bash -lc '
  export ROS_MASTER_URI=http://192.168.50.130:11311
  rostopic info /arm_controller/follow_joint_trajectory/goal
'
```

Only after a dry-run succeeds should a small, collision-free MoveIt motion be
executed, with the Fetch run-stop and an operator ready to cancel.
