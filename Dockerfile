# Use the official ROS Jazzy base image
FROM ros:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Add build arguments for user creation
ARG UID=1000
ARG GID=1000

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
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create user with same UID/GID as host user
RUN if getent passwd $UID >/dev/null; then \
        USER_NAME=$(getent passwd $UID | cut -d: -f1); \
        echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers; \
    else \
        if ! getent group $GID >/dev/null; then groupadd -g $GID hfp; fi && \
        useradd -u $UID -g $GID -m -s /bin/bash hfp && \
        echo "hfp ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
        USER_NAME=hfp; \
    fi

# Create a workspace directory
WORKDIR /ros2_ws

# Change ownership of workspace to the user
RUN if getent passwd $UID >/dev/null; then \
        USER_NAME=$(getent passwd $UID | cut -d: -f1); \
        chown -R $USER_NAME /ros2_ws; \
    else \
        chown -R hfp /ros2_ws; \
    fi

# Switch to the user
USER $UID

# Copy the ROS2 workspace files (if any) into the container
# ADD . /ros2_ws

# Source the ROS setup file and workspace setup if it exists
USER root
RUN if getent passwd $UID >/dev/null; then \
        USER_NAME=$(getent passwd $UID | cut -d: -f1); \
        echo "source /opt/ros/jazzy/setup.bash" >> /home/$USER_NAME/.bashrc && \
        echo "# Source workspace setup if it exists" >> /home/$USER_NAME/.bashrc && \
        echo "if [ -f /home/ROS2/ros2_ws/install/setup.bash ]; then" >> /home/$USER_NAME/.bashrc && \
        echo "    source /home/ROS2/ros2_ws/install/setup.bash" >> /home/$USER_NAME/.bashrc && \
        echo "fi" >> /home/$USER_NAME/.bashrc; \
    else \
        echo "source /opt/ros/jazzy/setup.bash" >> /home/hfp/.bashrc && \
        echo "# Source workspace setup if it exists" >> /home/hfp/.bashrc && \
        echo "if [ -f /home/ROS2/ros2_ws/install/setup.bash ]; then" >> /home/hfp/.bashrc && \
        echo "    source /home/ROS2/ros2_ws/install/setup.bash" >> /home/hfp/.bashrc && \
        echo "fi" >> /home/hfp/.bashrc; \
    fi
USER $UID

# Default command to keep the container running
CMD ["/bin/bash"]
