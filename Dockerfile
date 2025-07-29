# Use the official ROS Jazzy base image
FROM ros:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-colcon-common-extensions \
    git \
    curl \
    wget \
    ros-dev-tools \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    gedit \ 
    cmake \
    ninja-build \
    g++ \
    gcc \
    make \
    vim \
    nano \
    gdb \
    doxygen \
    lcov \
    gcovr \
    ccache \
    cppcheck \
    llvm \
    clang-format \
    clang-tidy \
    zip \
    unzip \
    tar \
    graphviz \
    libtinyxml2-dev \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /ros2_ws

# Copy the ROS2 workspace files (if any) into the container
# ADD . /ros2_ws

# Source the ROS setup file and workspace setup if it exists
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "# Source workspace setup if it exists" >> ~/.bashrc && \
    echo "if [ -f /home/ROS2/ros2_ws/install/setup.bash ]; then" >> ~/.bashrc && \
    echo "    source /home/ROS2/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc

# Default command to keep the container running
CMD ["/bin/bash"]
