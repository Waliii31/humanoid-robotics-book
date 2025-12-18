---
sidebar_position: 11
title: Bipedal Locomotion Fundamentals
---

# Bipedal Locomotion Fundamentals

## Learning Objectives

- Understand the mechanics of bipedal walking
- Implement Zero Moment Point (ZMP) stability criterion
- Calculate center of mass (CoM) trajectories
- Design simple walking gaits
- Apply balance control using feedback

## Introduction

Bipedal walking is one of the most challenging aspects of humanoid robotics. Unlike wheeled robots, bipeds are inherently unstable and must actively maintain balance at every moment.

## Stability Fundamentals

### Zero Moment Point (ZMP)

The **ZMP** is the point on the ground where the net moment from ground reaction forces equals zero.

**Stability Condition**: ZMP must remain within the support polygon (foot contact area).

```python
import numpy as np

def compute_zmp(com_position, com_acceleration, height, g=9.81):
    """
    Compute ZMP from center of mass state
    
    Args:
        com_position: [x, y, z] in meters
        com_acceleration: [ax, ay, az] in m/s^2
        height: CoM height from ground
        g: gravity (9.81 m/s^2)
    
    Returns:
        zmp: [x, y] position of ZMP
    """
    zmp_x = com_position[0] - (height / g) * com_acceleration[0]
    zmp_y = com_position[1] - (height / g) * com_acceleration[1]
    
    return np.array([zmp_x, zmp_y])

# Example
com_pos = np.array([0.05, 0.0, 0.8])  # Slightly forward, 80cm high
com_acc = np.array([0.1, 0.0, 0.0])   # Accelerating forward

zmp = compute_zmp(com_pos, com_acc, com_pos[2])
print(f"ZMP: {zmp}")  # Should be ahead of CoM projection
```

### Center of Pressure (CoP)

The actual point where ground reaction force is applied.

## Gait Cycle

### Walking Phases

1. **Double Support**: Both feet on ground (10-20% of cycle)
2. **Single Support**: One foot on ground (80-90% of cycle)
3. **Swing**: Free leg moves forward

```python
class GaitPhase(Enum):
    DOUBLE_SUPPORT_LEFT = 1
    SINGLE_SUPPORT_LEFT = 2
    DOUBLE_SUPPORT_RIGHT = 3
    SINGLE_SUPPORT_RIGHT = 4

class WalkingController:
    def __init__(self):
        self.phase = GaitPhase.DOUBLE_SUPPORT_LEFT  
        self.phase_time = 0.0
        
        # Timing parameters (seconds)
        self.double_support_duration = 0.1
        self.single_support_duration = 0.6
        self.step_duration = self.double_support_duration + self.single_support_duration
    
    def update(self, dt):
        self.phase_time += dt
        
        # Transition logic
        if self.phase == GaitPhase.DOUBLE_SUPPORT_LEFT:
            if self.phase_time > self.double_support_duration:
                self.phase = GaitPhase.SINGLE_SUPPORT_LEFT
                self.phase_time = 0.0
        
        elif self.phase == GaitPhase.SINGLE_SUPPORT_LEFT:
            if self.phase_time > self.single_support_duration:
                self.phase = GaitPhase.DOUBLE_SUPPORT_RIGHT
                self.phase_time = 0.0
        
        # ... continue cycle
```

## Linear Inverted Pendulum Model (LIPM)

Simplified model treating humanoid as inverted pendulum.

```python
class LIPM:
    def __init__(self, com_height=0.8, mass=50.0):
        self.h = com_height  # CoM height (constant)
        self.m = mass          # Robot mass
        self.g = 9.81
    
    def compute_com_acceleration(self, com_pos, zmp_ref):
        """
        Compute required CoM acceleration to achieve desired ZMP
        
        From dynamics: x_ddot = (g/h) * (x - zmp_x)
        """
        omega = np.sqrt(self.g / self.h)  # Natural frequency
        
        acc_x = omega**2 * (com_pos[0] - zmp_ref[0])
        acc_y = omega**2 * (com_pos[1] - zmp_ref[1])
        
        return np.array([acc_x, acc_y, 0.0])
    
    def step(self, com_pos, com_vel, zmp_target, dt):
        """Integrate dynamics one step"""
        com_acc = self.compute_com_acceleration(com_pos, zmp_target)
        
        # Euler integration
        com_vel_new = com_vel + com_acc * dt
        com_pos_new = com_pos + com_vel_new * dt
        
        return com_pos_new, com_vel_new
```

## Footstep Planning

```python
def plan_footsteps(start_pos, goal_pos, step_length=0.2, step_width=0.15):
    """
    Plan sequence of footsteps from start to goal
    
    Args:
        start_pos: [x, y] starting position
        goal_pos: [x, y] goal position
        step_length: forward step distance
        step_width: lateral distance between feet
    
    Returns:
        footsteps: List of (foot, position, orientation) tuples
    """
    footsteps = []
    
    direction = (goal_pos - start_pos) / np.linalg.norm(goal_pos - start_pos)
    distance = np.linalg.norm(goal_pos - start_pos)
    num_steps = int(distance / step_length)
    
    current_pos = start_pos.copy()
    
    for i in range(num_steps):
        # Alternate feet
        foot = "left" if i % 2 == 0 else "right"
        
        # Forward step
        step_pos = current_pos + direction * step_length
        
        # Lateral offset (left positive, right negative)
        lateral_offset = step_width / 2 if foot == "left" else -step_width / 2
        perpendicular = np.array([-direction[1], direction[0]])
        step_pos += perpendicular * lateral_offset
        
        # Orientation aligned with direction
        orientation = np.arctan2(direction[1], direction[0])
        
        footsteps.append((foot, step_pos, orientation))
        current_pos = step_pos
    
    return footsteps
```

## Balance Control

```python
class BalanceController:
    def __init__(self):
        # PID gains for ankle torque control
        self.kp = 1000.0
        self.kd = 100.0
        
    def compute_ankle_torque(self, desired_zmp, actual_zmp, zmp_velocity):
        """
        Compute ankle torques to drive ZMP to desired location
        """
        error = desired_zmp - actual_zmp
        
        # PD control
        torque_x = self.kp * error[0] - self.kd * zmp_velocity[0]
        torque_y = self.kp * error[1] - self.kd * zmp_velocity[1]
        
        return np.array([torque_x, torque_y])
```

## Lab Exercises

### Lab 11.1: ZMP Calculator
Implement ZMP calculation for various CoM trajectories and verify stability.

### Lab 11.2: 2D Biped Simulation
Create simple 2D biped in Gazebo, implement LIPM controller.

### Lab 11.3: Footstep Planner
Plan footsteps around obstacles, visualize in RViz2.

## Summary

✅ **ZMP criterion** determines bipedal stability  
✅ **LIPM** provides simple dynamics model for control design  
✅ **Gait cycle** alternates double and single support phases  
✅ **Balance control** uses feedback to maintain stability

---

**Next**: [Kinematics and Whole-Body Control](./kinematics-whole-body-control)
