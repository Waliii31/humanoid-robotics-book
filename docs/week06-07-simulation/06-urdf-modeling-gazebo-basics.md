---
sidebar_position: 6
title: URDF Modeling and Gazebo Basics
---

# URDF Modeling and Gazebo Basics

## Learning Objectives

- Write URDF descriptions for robot models
- Understand links, joints, and kinematic chains
- Launch robots in Gazebo Classic
- Configure physics properties and sensor plugins
- Integrate Gazebo with ROS 2

## Introduction

**URDF (Unified Robot Description Format)** is the standard for describing robot geometry and kinematics in ROS. **Gazebo** is the most widely-used physics simulator for robotics. Together, they enable rapid prototyping and testing before deploying to hardware.

## URDF Fundamentals

### Basic Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" 
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  
  <!--Joints connect links -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <link name="upper_arm">
    <!-- ... -->
  </link>
</robot>
```

## Xacro for Modularity

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <!-- Parameters -->
  <xacro:property name="arm_length" value="0.4"/>
  <xacro:property name="arm_radius" value="0.05"/>
  
  <!-- Macros for repeated structures -->
  <xacro:macro name="arm_link" params="prefix length">
    <link name="${prefix}_link">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${arm_radius}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>
  
  <!-- Use macro -->
  <xacro:arm_link prefix="upper_arm" length="0.4"/>
  <xacro:arm_link prefix="forearm" length="0.3"/>
  
</robot>
```

## Gazebo Integration

### Gazebo Plugins

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>

<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find humanoid_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### Spawning in Gazebo

```bash
# Launch Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# Spawn robot
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file robot.urdf
```

## Lab Exercises

### Lab 6.1: Build 7-DOF Arm URDF
Create complete URDF for humanoid arm with 7 joints.

### Lab 6.2: Add Sensors
Add camera and IMU to your robot model.

## Summary

✅ **URDF** defines robot kinematics and geometry  
✅ **Xacro** enables modular, parameterized robot descriptions  
✅ **Gazebo** provides physics-based simulation  
✅ **Plugins** integrate sensors and actuators with ROS 2

---

**Next**: [Gazebo-ROS Integration and Control](./gazebo-ros-integration-control)
