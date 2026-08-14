#!/bin/bash

# Resolve the repository root from this script location so that the setup does
# not depend on a particular user's home directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
FETCH_WS_ROOT="${FETCH_WS_ROOT:-$REPO_ROOT}"

# Runtime ROS network settings. Values supplied by the host environment take
# precedence; these defaults match the current Fetch network.
FETCH_ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.50.130:11311}"
FETCH_ROS_IP="${ROS_IP:-192.168.50.59}"
FETCH_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
FETCH_ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
FETCH_RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
FETCH_ROBOT_HOSTNAME="${FETCH_ROBOT_HOSTNAME:-fetch25}"
FETCH_ROBOT_IP="${FETCH_ROBOT_IP:-192.168.50.130}"

DOCKER_ROS_ENV_ARGS="--env ROS_MASTER_URI=$FETCH_ROS_MASTER_URI \
    --env ROS_IP=$FETCH_ROS_IP \
    --env ROS_DOMAIN_ID=$FETCH_ROS_DOMAIN_ID \
    --env ROS_LOCALHOST_ONLY=$FETCH_ROS_LOCALHOST_ONLY \
    --env RMW_IMPLEMENTATION=$FETCH_RMW_IMPLEMENTATION"

# Variables required for logging as a user with the same id as the user running this script
export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

ADDITIONAL_COMMANDS=""

# Variables for forwarding ssh agent into docker container
SSH_AUTH_ARGS=""
if [ ! -z $SSH_AUTH_SOCK ]; then
    DOCKER_SSH_AUTH_ARGS="-v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
fi

DOCKER_NETWORK_ARGS="--net host"
if [[ "$@" == *"--net "* ]]; then
    DOCKER_NETWORK_ARGS=""
fi

VOLUME_COMMANDS=""
for i in "$@"
do
case $i in
    # Receive ROS version from command line, through the --ros-distro argument
    --ros-distro=*)
    ROS_DISTRO="${i#*=}"
    echo "ROS distro set to: $ROS_DISTRO"
    shift # past argument=value
    ;;
    # Receive the --use-cuda argument
    --use-cuda)
    USE_CUDA=YES
    shift # past argument with no value
    ;;
    # Receive volumes to mount from command line, through the --volumes argument, each volume should be separated by a comma
    --volumes=*)
    for volume in $(echo ${i#*=} | tr "," "\n")
    do
        # If the path starts with ~, expand it to the user's home directory
        if [[ "$volume" == ~* ]]; then
            resolved_path="${volume/#\~/$HOME}"
        else
            resolved_path=$(realpath "$volume")
        fi
        folder_name=$(basename "$resolved_path")
        VOLUME_COMMANDS="$VOLUME_COMMANDS -v $resolved_path:/workspace/$folder_name"
    done
    shift # past argument=value
    ;;
    --name=*)
    # if the name is not empty, set the container name
    if [ -n "${i#*=}" ]; then
        CONTAINER_NAME="${i#*=}"
    fi
    shift # past argument=value
    ;;
    --l4t=*)
    # if l4t is not empty, set the l4t version
    if [ -n "${i#*=}" ]; then
        L4T_VERSION="${i#*=}"
        ADDITIONAL_COMMANDS+=" --runtime nvidia -v /sys/class/gpio:/sys/class/gpio \
        -v /sys/class/pwm:/sys/class/pwm --device /dev/spidev0.0:/dev/spidev0.0:rw \
        --device /dev/i2c-8:/dev/i2c-8:rw"
    fi
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done

IMAGE_NAME="emilianh/ros-$ROS_DISTRO"

if [ -z "$CONTAINER_NAME" ]; then
    CONTAINER_NAME="ros-$ROS_DISTRO-$USER"
fi

echo "Container name: $CONTAINER_NAME"
echo "Volumes to mount: $VOLUME_COMMANDS"
echo "ROS_MASTER_URI=$FETCH_ROS_MASTER_URI"
echo "ROS_IP=$FETCH_ROS_IP"

# Create the workspace mount points before Docker starts. This avoids Docker
# creating missing directories as root on a fresh checkout.
mkdir -p "$FETCH_WS_ROOT/ros1_ws" \
         "$FETCH_WS_ROOT/ros2_humble" \
         "$FETCH_WS_ROOT/ros1_bridge_ws"

# CHECK IF argument USE_CUDA is passed
DOCKER_GPU_ARGS=""
if [ -n "$USE_CUDA" ]; then
    DOCKER_GPU_ARGS="--gpus all"
    IMAGE_NAME="emilianh/ros-$ROS_DISTRO-cuda"
    echo "Using CUDA"
fi

if [ -n "$L4T_VERSION" ]; then
    IMAGE_NAME="emilianh/ros-$ROS_DISTRO-l4t$L4T_VERSION"
    echo "Using L4T $L4T_VERSION"
fi

IMAGE_NAME="$IMAGE_NAME:$USER"

MEMORY_LIMIT="--ulimit nofile=1024:524288"

DOCKER_COMMAND="docker run"

if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
    xhost +
fi

$DOCKER_COMMAND -it -d\
    $DOCKER_USER_ARGS \
    $DOCKER_GPU_ARGS \
    $DOCKER_SSH_AUTH_ARGS \
    $DOCKER_NETWORK_ARGS \
    $ADDITIONAL_COMMANDS \
    $MEMORY_LIMIT \
    $DOCKER_ROS_ENV_ARGS \
    --privileged \
    -p 11311:11311 \
    --add-host "$FETCH_ROBOT_HOSTNAME:$FETCH_ROBOT_IP" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v /dev:/dev \
    --device /dev/video0:/dev/video0 \
    -v "$FETCH_WS_ROOT/ros1_ws:/workspace" \
    -v "$FETCH_WS_ROOT/ros2_humble:/home/ros/ros2_humble" \
    -v "$FETCH_WS_ROOT/ros1_bridge_ws:/home/ros/ros1_bridge_ws" \
    $VOLUME_COMMANDS \
    -w /workspace \
    --name=$CONTAINER_NAME \
    $IMAGE_NAME \
    bash
