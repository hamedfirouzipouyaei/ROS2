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

## ROS 2 CLI Commands Reference

ROS 2 provides a comprehensive command-line interface for interacting with the ROS ecosystem. Here are the essential commands:

### Core Commands

#### `ros2 run`
Execute a specific node from a package:
```bash
ros2 run <package_name> <executable_name>
ros2 run my_py_pkg my_first_node
ros2 run demo_nodes_cpp talker
```

#### `ros2 launch`
Launch multiple nodes and configure complex systems:
```bash
ros2 launch <package_name> <launch_file>
ros2 launch demo_nodes_cpp talker_listener.launch.py
```

#### `ros2 pkg`
Package management and information:
```bash
ros2 pkg list                    # List all packages
ros2 pkg create <pkg_name>       # Create new package
ros2 pkg prefix <pkg_name>       # Show package installation path
ros2 pkg executables <pkg_name>  # List executables in package
```

### Communication Commands

#### `ros2 topic`
Manage and inspect ROS topics:
```bash
ros2 topic list                  # List all active topics
ros2 topic echo <topic_name>     # Display messages on topic
ros2 topic info <topic_name>     # Show topic information
ros2 topic hz <topic_name>       # Show publishing frequency
ros2 topic pub <topic> <msg_type> <data>  # Publish to topic
ros2 topic bw <topic_name>       # Show bandwidth usage
```

#### `ros2 service`
Interact with ROS services:
```bash
ros2 service list                # List all services
ros2 service call <service> <srv_type> <data>  # Call a service
ros2 service type <service>      # Show service type
ros2 service find <srv_type>     # Find services of specific type
```

#### `ros2 action`
Work with ROS actions (goal-oriented tasks):
```bash
ros2 action list                 # List all action servers
ros2 action info <action>        # Show action information
ros2 action send_goal <action> <action_type> <goal>  # Send goal
```

### Node Management

#### `ros2 node`
Inspect and manage running nodes:
```bash
ros2 node list                   # List all running nodes
ros2 node info <node_name>       # Show node details
ros2 node kill <node_name>       # Terminate a node
```

### Interface Commands

#### `ros2 interface`
Examine message, service, and action definitions:
```bash
ros2 interface list              # List all interfaces
ros2 interface show <interface>  # Show interface definition
ros2 interface package <pkg>     # Show interfaces in package
ros2 interface packages          # List packages with interfaces
```

### Parameter Management

#### `ros2 param`
Configure node parameters:
```bash
ros2 param list                  # List all parameters
ros2 param get <node> <param>    # Get parameter value
ros2 param set <node> <param> <value>  # Set parameter value
ros2 param dump <node>           # Export all parameters
ros2 param load <node> <file>    # Load parameters from file
```

### Data Recording and Playback

#### `ros2 bag`
Record and replay ROS data:
```bash
ros2 bag record <topic1> <topic2>  # Record specific topics
ros2 bag record -a               # Record all topics
ros2 bag play <bag_file>         # Replay recorded data
ros2 bag info <bag_file>         # Show bag information
ros2 bag reindex <bag_file>      # Reindex corrupted bag
```

### System Information

#### `ros2 doctor`
Diagnose ROS 2 setup and configuration:
```bash
ros2 doctor                      # Check system health
ros2 doctor --report             # Generate detailed report
```

#### `ros2 wtf`
Show comprehensive system status (What's This For?):
```bash
ros2 wtf                         # Display system overview
```

### Debugging and Development

#### `ros2 component`
Manage component-based nodes:
```bash
ros2 component list              # List component containers
ros2 component types             # Show available component types
ros2 component load <container> <component>  # Load component
```

#### `ros2 lifecycle`
Control lifecycle nodes:
```bash
ros2 lifecycle list              # List lifecycle nodes
ros2 lifecycle get <node>        # Get current state
ros2 lifecycle set <node> <state>  # Change node state
```

### Visualization and GUI Tools

#### `rqt`
RQT is the main GUI framework for ROS 2, providing a plugin-based interface for various tools:

```bash
rqt                              # Launch RQT with plugin selection
rqt --list-plugins               # List available plugins
rqt --standalone <plugin_name>   # Launch specific plugin directly
```

**Common RQT Plugins:**
- **rqt_graph**: Visualize node and topic connections
- **rqt_plot**: Real-time data plotting
- **rqt_console**: View log messages
- **rqt_service_caller**: Call services with GUI
- **rqt_topic**: Topic monitoring and publishing
- **rqt_bag**: Bag file viewer and player
- **rqt_tf_tree**: Transform tree visualization
- **rqt_image_view**: Display camera feeds

#### `rqt_graph`
Visualize the ROS computation graph (nodes, topics, services):

```bash
rqt_graph                        # Launch node/topic graph
rqt_graph --force-discover       # Force discovery of all connections
```

**Features:**
- **Node View**: Shows running nodes and their connections
- **Topic View**: Displays topics and their publishers/subscribers
- **Service View**: Visualizes service connections
- **Action View**: Shows action servers and clients

**Usage Examples:**

1. **Basic Graph Visualization:**
   ```bash
   # Terminal 1: Start some demo nodes
   ros2 run demo_nodes_cpp talker
   
   # Terminal 2: Start listener
   ros2 run demo_nodes_cpp listener
   
   # Terminal 3: Launch graph viewer
   rqt_graph
   ```

2. **Analyze Complex Systems:**
   ```bash
   # Launch a complex system (e.g., navigation stack)
   ros2 launch nav2_bringup tb3_simulation_launch.py
   
   # View the computation graph
   rqt_graph
   ```

3. **Filter Graph View:**
   - Use the GUI filters to show/hide:
     - Dead sinks (nodes with no outputs)
     - Leaf topics (topics with no subscribers)
     - Debug nodes
     - Specific namespaces

#### Other Useful RQT Tools

**rqt_plot - Real-time Data Visualization:**
```bash
rqt_plot                         # Launch plotting interface
rqt_plot /topic_name/field       # Plot specific message field
rqt_plot /turtle1/pose/x /turtle1/pose/y  # Plot multiple fields
```

**rqt_console - Log Message Viewer:**
```bash
rqt_console                      # View system log messages
```
- Filter by severity (Debug, Info, Warn, Error, Fatal)
- Filter by node name
- Search log messages

**rqt_service_caller - Service GUI:**
```bash
rqt_service_caller               # GUI for calling services
```

**rqt_topic - Topic Monitor:**
```bash
rqt_topic                        # Monitor and publish to topics
```

**rqt_bag - Bag File Viewer:**
```bash
rqt_bag <bag_file>               # View and analyze bag files
```

**rqt_image_view - Camera Feed Viewer:**
```bash
rqt_image_view                   # Display camera topics
```

### RQT Integration Examples

**Complete Debugging Session:**
```bash
# 1. Start your nodes
ros2 run my_package my_node

# 2. Check the computation graph
rqt_graph

# 3. Monitor log messages
rqt_console

# 4. Plot sensor data
rqt_plot /sensor_data/temperature

# 5. View camera feed (if available)
rqt_image_view

# 6. All-in-one dashboard
rqt  # Then add multiple plugins in tabs/docks
```

**Custom RQT Perspective:**
RQT allows you to save custom layouts (perspectives) with multiple plugins:
1. Launch `rqt`
2. Add desired plugins (Plugins menu)
3. Arrange them in tabs or docked windows
4. Save perspective: Perspectives → Create Perspective
5. Load later: Perspectives → Import/Export

### Examples

Here are some practical examples:

```bash
# Start a talker node
ros2 run demo_nodes_cpp talker

# In another terminal, listen to the topic
ros2 topic echo /chatter

# Check topic information
ros2 topic info /chatter

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Record some data
ros2 bag record /chatter

# Play it back later
ros2 bag play rosbag2_xxx
```

### Pro Tips

- Use tab completion for commands and topics: `ros2 topic <TAB><TAB>`
- Most commands support `--help` flag for detailed usage
- Use `ros2 topic list | grep <pattern>` to filter topics
- Combine with shell tools: `ros2 node list | wc -l` (count running nodes)

## Contributing

1. Create your feature branch
2. Make your changes in the appropriate package
3. Build and test your changes
4. Submit a pull request

## License

TODO: Add license information
