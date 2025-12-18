---
sidebar_position: 5
title: Parameters, Launch Files, and TF2
---

# Parameters, Launch Files, and TF2

## Learning Objectives

- Manage robot parameters dynamically with ROS 2
- Create complex launch files for multi-node systems
- Understand TF2 for coordinate frame transformations  
- Visualize robot state in RViz2
- Build modular, configurable robot systems

## Introduction

Modern robot systems require configuration management, multi-node orchestration, and spatial reasoning. This chapter covers three essential tools: **parameters** for configuration, **launch files** for system composition, and **TF2** for coordinate transforms.

## Parameters

### Declaring Parameters

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        
        # Declare with default values
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('robot_name', 'humanoid_01')
        self.declare_parameter('joint_limits', [1.57, 1.57, 1.57])
        
        # Get values
        self.max_speed = self.get_parameter('max_speed').value
        self.robot_name = self.get_parameter('robot_name').value
```

### Parameter Callbacks

```python
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class DynamicNode(Node):
    def __init__(self):
        super().__init__('dynamic_node')
        
        # Declare with descriptor (range limits)
        descriptor = ParameterDescriptor(
            description='Maximum speed in m/s',
            floating_point_range=[
                FloatingPointRange(from_value=0.0, to_value=2.0, step=0.1)
            ]
        )
        self.declare_parameter('max_speed', 1.0, descriptor)
        
        # Register callback
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_speed':
                self.get_logger().info(f'Speed changed to {param.value}')
        
        return SetParametersResult(successful=True)
```

## Launch Files

### Python Launch File

```python
# launch/humanoid_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'robot_name',
            default_value='humanoid_01',
            description='Name of the robot'
        ),
        
        # Joint state publisher
        Node(
            package='humanoid_control',
            executable='joint_publisher',
            name='joint_publisher',
            parameters=[{
                'num_joints': 12,
                'robot_name': LaunchConfiguration('robot_name')
            }],
            remappings=[
                ('joint_commands', '/robot/joint_commands')
            ]
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_urdf}]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file]
        ),
    ])
```

### Launch with Arguments

```bash
ros2 launch humanoid_control humanoid_bringup.launch.py robot_name:=atlas_v2
```

## TF2: Transform Library

### Broadcasting Transforms

```python
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.broadcast_tf)  # 100 Hz
        
    def broadcast_tf(self):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        # Position
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.5
        t.transform.translation.z = 0.0
        
        # Rotation (quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(math.pi/4)
        t.transform.rotation.w = math.cos(math.pi/4)
        
        self.tf_broadcaster.sendTransform(t)
```

### Looking Up Transforms

```python
from tf2_ros import TransformListener, Buffer

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.lookup_transform)
    
    def lookup_transform(self):
        try:
            # Look up transform from world to end_effector
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'world',
                'end_effector',
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.get_logger().info(
                f'End effector at: ({trans.transform.translation.x:.2f}, '
                f'{trans.transform.translation.y:.2f}, '
                f'{trans.transform.translation.z:.2f})'
            )
            
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')
```

## URDF and robot_state_publisher

### Simple URDF

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.1"/>
      </geometry>
    </visual>
  </link>
  
  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

## Visualizing in RViz2

```bash
# Launch RViz with robot model
ros2 launch robot_state_publisher demo.launch.py
```

## Hands-On Lab

### Lab 5.1: Create Parameterized Controller
Build a controller node with configurable gains that can be adjusted at runtime.

### Lab 5.2: Multi-Robot Launch
Create launch file that spawns 3 robot instances with different namespaces.

### Lab 5.3: TF Tree
Build complete TF tree for 7-DOF arm and visualize in RViz2.

## Summary

✅ **Parameters** enable runtime configuration without recompiling  
✅ **Launch files** orchestrate complex multi-node systems  
✅ **TF2** manages coordinate transformations across robot links  
✅ **robot_state_publisher** publishes TF from URDF + joint states  
✅ **RViz2** visualizes robot state and sensor data

---

**Next**: [URDF Modeling and Gazebo Basics](../week06-07-simulation/urdf-modeling-gazebo-basics)
