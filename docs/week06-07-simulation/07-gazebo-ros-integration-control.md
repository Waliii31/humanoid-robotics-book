---
sidebar_position: 7
title: Gazebo-ROS Integration and Control
---

# Gazebo-ROS Integration and Control

## Learning Objectives

- Integrate Gazebo simulations with ROS 2
- Implement ros2_control framework for robot actuation
- Use joint trajectory controllers
- Simulate sensor data publishing
- Create teleoperation systems

## Introduction

**ros2_control** is the standard framework for robot control in ROS 2. It provides a hardware-agnostic interface for controlling joints, whether in simulation or on real robots.

## ros2_control Framework

### Controller Configuration

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
      - wrist_4_joint
    
    command_interfaces:
      - position
    
    state_interfaces:
      - position
      - velocity
```

### Launch with Controllers

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_controllers],
            output='both',
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
        ),
    ])
```

## Sending Trajectories

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
    
    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', ...]
        
        # Waypoint 1
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.5, 0.3, 0.0, 0.0, 0.0, 0.0]
        point1.time_from_start.sec = 2
        
        #Waypoint 2
        point2 = JointTrajectoryPoint()
        point2.positions = [1.0, 1.0, 0.5, 0.5, 0.5, 0.0, 0.0]
        point2.time_from_start.sec = 4
        
        msg.points = [point1, point2]
        self.pub.publish(msg)
```

## Summary

✅ **ros2_control** provides hardware abstraction for robot control  
✅ **Joint trajectory controllers** execute motion plans  
✅ **Gazebo integration** enables testing before hardware deployment

---

**Next**: [Isaac Sim Introduction](../week08-10-nvidia-isaac/isaac-sim-introduction)
