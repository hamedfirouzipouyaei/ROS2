#!/bin/bash

# This script builds the ROS2 workspace inside the docker container

WORKSPACE_DIR="/home/ROS2/ros2_ws"

echo "🚀 Building ROS2 workspace at ${WORKSPACE_DIR}"

# Create workspace if it doesn't exist
if [ ! -d "${WORKSPACE_DIR}" ]; then
    echo "Creating workspace directory: ${WORKSPACE_DIR}"
    mkdir -p "${WORKSPACE_DIR}/src"
fi

# Change to workspace directory
cd "${WORKSPACE_DIR}"

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Build the workspace
echo "Building workspace with colcon..."
colcon build

# Source the workspace setup
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
    echo "✅ Workspace built and sourced successfully!"
    echo "The workspace setup will be automatically sourced in future terminal sessions."
else
    echo "❌ Build failed or setup.bash not found"
    exit 1
fi
