## Stage 1: Setting up the ROS Docker Environment

As the first step, I prepared a Docker-based ROS development environment based
on the [`ros-docker`](https://github.com/EmilianoHFlores/ros-docker) project. I
then integrated the Docker files required for this project into this
repository, so the complete setup can be cloned from a single repository.

The main purpose of this setup is to isolate ROS 1 Noetic from the host system. The host machine runs Ubuntu 22.04 with ROS 2 Humble, while ROS 1 Noetic runs inside a Docker container.

### Environment

- Host OS: Ubuntu 22.04
- Host ROS: ROS 2 Humble
- Docker container: ROS 1 Noetic
- Target robot: Fetch
- Host architecture: x86_64 Linux

### Customizing `Dockerfile.noetic`

I customized `Dockerfile.noetic` based on the `althack/ros:noetic-full` image.
The customization includes refreshing the ROS archive key, installing the
development tools and ROS packages required for this project, adding Gazebo
Classic support, and preparing the `/workspace` directory for the ROS 1
workspace.

The resulting `Dockerfile.noetic` is:

```dockerfile
FROM althack/ros:noetic-full
LABEL maintainer="Emiliano Flores <joemilianofm@gmail.com>"

# The base image contains an expired ROS archive signing key. Bootstrap the
# current keyring before running apt-get update against packages.ros.org.
RUN rm -f /etc/apt/sources.list.d/ros*.list && \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends ca-certificates curl gnupg && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
      > /etc/apt/sources.list.d/ros1.list

# Install dependencies and additional ROS tools.
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
    build-essential \
    ffmpeg libsm6 libxext6 autoconf libtool mesa-utils \
    terminator nano git wget curl iputils-ping \
    libcanberra-gtk-module libcanberra-gtk3-module \
    ros-dev-tools python3-catkin-tools python3-vcstool python3-tk \
    net-tools \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-moveit ros-noetic-navigation

# Gazebo Classic packages are provided by the ROS/Ubuntu repositories. The
# old get.gazebosim.org bootstrap script is deprecated and exits with failure.
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN mkdir /workspace
RUN chown -R ros:ros /workspace

ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
```

### Exposing the ROS Master Port

To make the ROS 1 master port accessible from outside the container, I added
the following port mapping to the `DOCKER_COMMAND` in `run.bash`:

```bash
-p 11311:11311
```

This maps port `11311` on the host to port `11311` inside the container and
exposes the standard ROS master port. ROS 1 communication also depends on
`ROS_MASTER_URI`, `ROS_IP`, and the container network configuration.

### Configuring the Container Environment

Inside the Docker container, I added the following settings to the container
user's `~/.bashrc`. These settings configure the ROS 1 master connection, ROS 2
DDS behavior, and the hostname used for the Fetch robot:

```bash
export ROS_MASTER_URI=http://192.168.50.130:11311
export ROS_IP=192.168.50.59
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
echo "192.168.50.130 fetch25" | sudo tee -a /etc/hosts
```

### Building the Docker Image

The Docker configuration from `ros-docker` is now included in this repository
under `docker/`, together with the root `Makefile`. From the repository root, I
built the ROS Noetic Docker image:

```bash
make noetic.build
```

After building the image, I created the container:

```bash
make noetic.create
```

This command creates and starts the container. If the container already exists
but is stopped, it can be started with:

The container can be started with:

```bash
make noetic.up
```

To open a terminal inside the running container:

```bash
make noetic.shell
```

The Docker run script derives the repository root from its own location, so it
does not depend on the original `/home/a/fetch_moveit_ws` path. The ROS 1,
ROS 2, and `ros1_bridge` workspaces are mounted from the current checkout.

### Installing ROS 2 Humble from Source in the Docker Container

Before installing ROS 2, I cleared the ROS 1 environment variables from the
shell. This was necessary to prevent ROS 1 Noetic and ROS 2 Humble paths from
being mixed during the source build.

I used the following commands before starting the ROS 2 setup:

```bash
unset ROS_VERSION ROS_DISTRO ROS_PYTHON_VERSION
unset ROS_ROOT ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
```

I then checked that no ROS environment remained in the shell:

```bash
printenv | grep -i ROS
```

The command should not print any output. The ROS network variables such as
`ROS_MASTER_URI` and `ROS_IP` were configured again later when starting the
ROS 1 runtime environment.

Because the container is used to build the ROS 1--ROS 2 bridge, I also
installed ROS 2 Humble from source inside the container. I followed the
official [ROS 2 Humble Ubuntu Development
Setup](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
to prepare the development environment, obtain the Humble source repositories,
install their dependencies, and build the ROS 2 workspace.

This ROS 2 installation is kept separate from the host-side ROS 2
installation and is used inside the container as part of the build environment
for `ros1_bridge`.

### Building `ros1_bridge`

After installing ROS 2 Humble, I built `ros1_bridge` inside the same Docker
container. This bridge is required to exchange compatible topics and messages
between the ROS 1 Noetic and ROS 2 Humble environments.

Before building the bridge, I cleared the Python environment variables to avoid
mixing Python paths from the ROS 1 and ROS 2 installations. I also increased the
number of callback-processing threads used by the dynamic bridge from 1 to 4.
I then sourced both ROS distributions and built only the `ros1_bridge` package
with testing disabled.

```bash
cd ~

mkdir -p ~/ros1_bridge_ws/src
cd ~/ros1_bridge_ws/src

git clone https://github.com/ros2/ros1_bridge.git

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

source ~/ros1_bridge_ws/install/local_setup.bash
```

The resulting bridge is used later to connect the ROS 1 Fetch topics with the
ROS 2 MoveIt environment.

### Making the Fetch Meshes Portable

The original Fetch URDF referenced the mesh files through a machine-specific
absolute path. To make the robot description portable, I copied the Fetch mesh
assets into the `fetch_description_local` package and changed the URDF mesh
references to use the `package://fetch_description_local/meshes/` URI format.

The package now installs both the URDF files and the mesh files. The origin and
license information for these third-party assets is recorded in
`src/fetch_description_local/THIRD_PARTY_NOTICES.md`.

At this stage, I verified that the ROS 1 Noetic Docker environment could be
built and started, and that a source-based ROS 2 Humble workspace could be
created inside the container and that `ros1_bridge` could be built in the same
environment.

The Fetch-specific ROS packages and the MoveIt 2 configuration are added in the
following stages.
