# ROS 2 Workspace

This is a ROS 2 workspace containing custom packages for ROS Jazzy.

## Setup

### Prerequisites
- Docker with NVIDIA Container Toolkit (for GPU support)
- ROS 2 Jazzy

### Quick Start with Docker

1. **Build the Docker image:**
   ```bash
   ./build.sh
   ```

2. **Run the container:**
   ```bash
   ./run.sh
   ```

3. **Build the workspace (inside container):**
   ```bash
   /home/ROS2/build_workspace.sh
   ```

### Manual Setup (without Docker)

1. **Source ROS 2:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. **Build the workspace:**
   ```bash
   cd ros2_ws
   colcon build
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Packages

### my_cpp_pkg
A C++ package demonstrating basic ROS 2 functionality.

## ROS 2 Architecture Overview

<img src="docs/images/ROS2.png" alt="ROS 2 Nodes Architecture" width="75%">

This workspace is designed to implement a multi-package ROS 2 system with the following architecture:

### Package Structure & Node Communication

The system consists of three main packages, each containing specialized nodes:

#### 1. **Motion Planning Package**
- **Motion Planning Node**: Core path planning algorithms
- **Path Correction Node**: Real-time path adjustments based on feedback
- Communicates with camera package for environmental awareness
- Sends commands to hardware control package

#### 2. **Camera Package** 
- **Camera Driver Node**: Hardware interface for camera sensors
- **Image Processing Node**: Computer vision algorithms for environment perception
- Additional nodes (...)  for extended functionality
- Provides visual feedback to motion planning for obstacle avoidance and navigation

#### 3. **Hardware Control Package**
- **Hardware Driver Node**: Low-level hardware interface
- **State Publisher Node**: Publishes robot state information
- Receives motion commands from planning package
- Provides feedback to motion planning for closed-loop control

### Communication Flow

1. **Sensor Data**: Camera driver captures environmental data
2. **Processing**: Image processing node extracts relevant information
3. **Planning**: Motion planning uses visual data to generate paths
4. **Correction**: Path correction adjusts plans based on real-time feedback
5. **Execution**: Hardware driver executes motion commands
6. **Feedback**: State publisher provides current robot state back to planners

This distributed architecture ensures modularity, fault tolerance, and scalability while maintaining real-time performance for robotic applications.

## Workspace Structure

```
ros2_ws/
├── src/
│   └── my_cpp_pkg/
├── build/
├── install/
└── log/
```

## Development

- Source code is located in `ros2_ws/src/`
- Build artifacts are in `ros2_ws/build/` and `ros2_ws/install/`
- Use `colcon build` to build all packages
- Use `colcon build --packages-select <package_name>` to build specific packages

## Docker Environment

The Docker container includes:
- ROS 2 Jazzy
- Development tools (cmake, ninja, gcc, clang, etc.)
- Code analysis tools (cppcheck, clang-tidy, etc.)
- Debugging tools (gdb)
- Documentation tools (doxygen)
- Text editors (vim, nano, gedit)
- GPU support (NVIDIA)

## Contributing

1. Create your feature branch
2. Make your changes in the appropriate package
3. Build and test your changes
4. Submit a pull request

## License

TODO: Add license information
