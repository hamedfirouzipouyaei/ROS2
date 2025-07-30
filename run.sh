#!/bin/bash
# docker run -it --rm -v "$(pwd)":/ros2_ws ros-jazzy

#!/bin/bash
set -ex

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo "Docker could not be found. Please install Docker." >&2
    exit 1
fi

# Check for NVIDIA runtime (if using GPU)
if ! docker info | grep -q 'Runtimes:.*nvidia'; then
    echo "NVIDIA Docker runtime not found. GPU support may not work." >&2
fi

# Check DISPLAY variable for GUI apps
if [ -z "$DISPLAY" ]; then
    echo "DISPLAY variable is not set. GUI apps may not work." >&2
fi

# Ensure workspace directory exists
if [ ! -d "/home/hfp/Github/ROS2/" ]; then
    echo "Workspace directory /home/hfp/Github/ROS2/ does not exist. Creating it." >&2
    mkdir -p /home/hfp/Github/ROS2/
fi

# Clean up any existing container named ROS2_h
docker stop ROS2_h 2>/dev/null || true
docker rm -f ROS2_h 2>/dev/null || true

echo "run the following command to start interactive container"
echo "docker exec -it ROS2_h bash"

# Define the IMAGE you want to run
IMAGE_NAME="ros2:h"

# Start the container
docker run \
    --rm \
    -ti \
    --gpus all \
    --privileged \
    --env=DISPLAY=$DISPLAY \
    --env=XAUTHORITY=/tmp/.Xauthority \
    --volume=/home/hfp/Github/ROS2/:/home/ROS2 \
    --volume=$XAUTHORITY:/tmp/.Xauthority:rw \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --name=ROS2_h \
    --hostname=ROS2_h \
    --network=host \
    --add-host=ROS2_h:127.0.1.1 \
    --workdir=/home/ROS2 \
    "${IMAGE_NAME}" \
    ${@:-bash}