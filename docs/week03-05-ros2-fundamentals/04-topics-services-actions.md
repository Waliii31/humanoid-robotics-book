---
sidebar_position: 4
title: Topics, Services, and Actions
---

# Topics, Services, and Actions

## Learning Objectives

- Implement publish-subscribe communication with topics
- Create service servers and clients for request-reply patterns
- Use action servers for long-running tasks with feedback
- Select appropriate communication patterns for different scenarios
- Create custom message and service definitions

## Introduction

ROS 2 provides three primary communication patterns: **topics** (publish-subscribe), **services** (request-reply), and **actions** (goal-feedback-result). Each serves a different purpose in robot systems.

## Topics: Publish-Subscribe

### When to Use Topics
- Continuous data streams (sensor data, robot state)
- One-to-many or many-to-many communication
- Fire-and-forget semantics

### Publisher Example

```python
from geometry_msgs.msg import Twist

class RobotController(Node):
    def__init__(self):
        super().__init__('robot_controller')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def move_forward(self, speed=0.5):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
```

## Services: Request-Reply

### When to Use Services
- One-shot operations (reset, calibration)
- Synchronous operations where you need confirmation
- Configuration changes

### Service Server

```python
from example_interfaces.srv import SetBool

class CalibrateService(Node):
    def __init__(self):
        super().__init__('calibrate_service')
        self.srv = self.create_service(
            SetBool,
            'calibrate_robot',
            self.calibrate_callback
        )
    
    def calibrate_callback(self, request, response):
        if request.data:
            # Perform calibration
            self.get_logger().info('Calibrating...')
            # ... calibration logic ...
            response.success = True
            response.message = 'Calibration complete'
        else:
            response.success = False
            response.message = 'Calibration canceled'
        
        return response
```

### Service Client

```python
class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.cli = self.create_client(SetBool, 'calibrate_robot')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def send_request(self, calibrate=True):
        request = SetBool.Request()
        request.data = calibrate
        
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()
```

## Actions: Long-Running Tasks

### When to Use Actions
- Tasks that take time (navigation, grasping)
- Need periodic feedback
- Should be cancellable

### Action Server Example

```python
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

class TrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('trajectory_action_server')
        
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            self.execute_callback
        )
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing trajectory...')
        
        feedback_msg = FollowJointTrajectory.Feedback()
        
        # Execute trajectory points
        for i, point in enumerate(goal_handle.request.trajectory.points):
            # Send feedback
            feedback_msg.actual = point
            goal_handle.publish_feedback(feedback_msg)
            
            # Execute motion
            self.move_to_point(point)
            time.sleep(0.1)
            
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
        
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result
```

## Custom Messages

### Define Custom Message

```
# Create humanoid_msgs package
ros2 pkg create --build-type ament_cmake humanoid_msgs

# Create msg/JointCommand.msg
float64[] positions
float64[] velocities
float64[] efforts
builtin_interfaces/Time timestamp
```

### Update CMakeLists.txt

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/JointCommand.msg"
    DEPENDENCIES builtin_interfaces
)
```

## Lab Exercises

### Lab 4.1: Create Echo Service
Build a service that echoes back any string sent to it.

### Lab 4.2: Multi-Step Action
Create an action for "pick and place" with feedback at each step.

## Summary

✅ **Topics** for continuous data streams (sensor readings, commands)  
✅ **Services** for one-shot synchronous operations (calibration, reset)  
✅ **Actions** for long-running cancellable tasks (navigation, trajectories)  
✅ Custom messages for domain-specific data types

---

**Next**: [Parameters, Launch Files, and TF2](./parameters-launch-tf2)
