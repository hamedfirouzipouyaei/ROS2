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
