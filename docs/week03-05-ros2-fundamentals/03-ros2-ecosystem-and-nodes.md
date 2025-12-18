---
sidebar_position: 3
title: ROS 2 Ecosystem and Nodes
---

# ROS 2 Ecosystem and Nodes

## Learning Objectives

- Install and configure ROS 2 Humble on Ubuntu 22.04
- Understand the DDS middleware and its role in ROS 2
- Create basic ROS 2 nodes in both Python and C++
- Use ROS 2 command-line tools for debugging and introspection
- Build and manage ROS 2 packages with colcon

## Introduction

Welcome to **ROS 2 (Robot Operating System 2)**—the industry-standard middleware for building robot applications. Despite its name, ROS 2 isn't an operating system; it's a powerful framework that provides communication infrastructure, hardware abstraction, and a rich ecosystem of tools and libraries.

This week marks the beginning of your hands-on journey. By the end of this chapter, you'll have ROS 2 running on your system and will have created your first robot control nodes.

## ROS 2 vs ROS 1

### Key Improvements

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Architecture** | Master-based (XMLRPC) | Distributed (DDS) |
| **Real-Time** | Limited | Native RT support |
| **Security** | None | DDS Security |
| **Platforms** | Linux only | Linux, Windows, macOS |
| **Python** | Python 2.7 | Python 3.x |
| **Build System** | catkin | ament (CMake/Python) |

### Why ROS 2 for Humanoids?

1. **Real-time control**: Critical for balance and motor control
2. **No single point of failure**: Distributed discovery
3. **Production-ready**: Used in commercial robots
4. **Better tooling**: Improved debugging and visualization

## Installation

### Step 1: Install ROS 2 Humble

```bash
# Set up sources
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Verify Installation

```bash
# Check ROS 2 version
ros2 --version
# Should output: ros2 cli version 0.18.x

# Run demo
ros2 run demo_nodes_cpp talker
# In another terminal:
ros2 run demo_nodes_py listener
```

## Understanding DDS Middleware

### What is DDS?

**DDS (Data Distribution Service)** is an OMG standard for real-time, distributed systems.

**Key Concepts**:
- **Automatic discovery**: Nodes find each other without a master
- **Pub-Sub architecture**: Decoupled communication
- **QoS policies**: Configure reliability, latency, durability

### DDS Implementations

```bash
# Default: Fast-DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Alternative: Cyclone DDS (good performance)
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Creating Your First Node (Python)

### Package Structure

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python \
    --dependencies rclpy std_msgs \
    humanoid_control
```

### Simple Publisher Node

```python
# humanoid_control/humanoid_control/joint_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        
        # Create publisher
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10  # QoS depth
        )
        
        # Create timer (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_joint_commands)
        
        # Parameters
        self.declare_parameter('num_joints', 7)
        self.num_joints = self.get_parameter('num_joints').value
        
        self.get_logger().info(f'Publishing commands for {self.num_joints} joints')
    
    def publish_joint_commands(self):
        msg = Float64MultiArray()
        
        # Simple sinusoidal motion
        import math
        t = self.get_clock().now().seconds_nanoseconds()[0]
        msg.data = [math.sin(t + i) for i in range(self.num_joints)]
        
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Update setup.py

```python
from setuptools import setup

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'joint_publisher = humanoid_control.joint_publisher:main',
        ],
    },
)
```

### Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_control
source install/setup.bash

# Run node
ros2 run humanoid_control joint_publisher

# With parameters
ros2 run humanoid_control joint_publisher --ros-args -p num_joints:=10
```

## Creating a C++ Node

### C++ Publisher

```cpp
// humanoid_control/src/joint_publisher.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

class JointPublisher : public rclcpp::Node {
public:
    JointPublisher() : Node("joint_publisher_cpp") {
        // Declare parameter
        this->declare_parameter("num_joints", 7);
        num_joints_ = this->get_parameter("num_joints").as_int();
        
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "joint_commands", 10
        );
        
        // Create timer (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JointPublisher::publishJointCommands, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Publishing commands for %d joints", num_joints_);
    }

private:
    void publishJointCommands() {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(num_joints_);
        
        auto now = this->now();
        double t = now.seconds();
        
        for (int i = 0; i < num_joints_; ++i) {
            msg.data[i] = std::sin(t + i);
        }
        
        publisher_->publish(msg);
    }
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int num_joints_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(humanoid_control)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(joint_publisher_cpp src/joint_publisher.cpp)
ament_target_dependencies(joint_publisher_cpp rclcpp std_msgs)

install(TARGETS
    joint_publisher_cpp
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

## ROS 2 Command-Line Tools

### Essential Commands

```bash
# List running nodes
ros2 node list

# Node information
ros2 node info /joint_publisher

# List topics
ros2 topic list

# Echo topic data
ros2 topic echo /joint_commands

# Publish manually
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \
    "data: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]"

# Topic frequency
ros2 topic hz /joint_commands

# Topic info
ros2 topic info /joint_commands

# Parameter operations
ros2 param list
ros2 param get /joint_publisher num_joints
ros2 param set /joint_publisher num_joints 10
```

### Introspection with rqt

```bash
# Launch rqt
rqt

# Useful plugins:
# - Node Graph: Visualize nodes and topics
# - Topic Monitor: View message data
# - Parameter Reconfigure: Adjust parameters live
# - Console: View log messages
```

## Hands-On Lab

### Lab 3.1: Create Subscriber Node

Create a subscriber that listens to `/joint_commands` and prints statistics:

```python
class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_callback,
            10
        )
        self.count = 0
        
    def joint_callback(self, msg):
        self.count += 1
        mean = sum(msg.data) / len(msg.data)
        max_val = max(msg.data)
        
        self.get_logger().info(
            f'Received {self.count} messages. Mean: {mean:.3f}, Max: {max_val:.3f}'
        )
```

### Lab 3.2: Multi-Node System

Build a 3-node system:
1. **Sensor Simulator**: Publishes fake IMU data
2. **Filter Node**: Applies smoothing filter
3. **Visualizer**: Logs filtered data

## Summary

✅ **ROS 2 Humble** installed with DDS middleware for distributed communication

✅ **Nodes** are the basic computation units in ROS 2 (Python or C++)

✅ **Publishers** send messages on topics; **Subscribers** receive them

✅ **Command-line tools** (`ros2 node`, `ros2 topic`) enable powerful debugging

✅ **colcon** builds ROS 2 packages efficiently

---

**Next**: [Topics, Services, and Actions](./topics-services-actions)
