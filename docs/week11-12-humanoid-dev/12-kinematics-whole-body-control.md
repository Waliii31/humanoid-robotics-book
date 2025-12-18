---
sidebar_position: 12
title: Kinematics and Whole-Body Control
---

# Kinematics and Whole-Body Control

## Learning Objectives

- Solve forward and inverse kinematics for humanoid arms
- Implement whole-body inverse kinematics with multiple constraints
- Use MoveIt2 for motion planning
- Coordinate multiple simultaneous task objectives
- Apply redundancy resolution techniques

## Introduction

Humanoid robots have 30-50+ degrees of freedom. **Whole-body control** coordinates all these joints simultaneously to achieve multiple objectives: maintaining balance, moving the arms, looking at targets, and avoiding obstacles.

## Forward Kinematics

```python
import modern_robotics as mr
import numpy as np

def forward_kinematics_arm(joint_angles):
    """
    7-DOF arm forward kinematics using Product of Exponentials (PoE)
    """
    # Home configuration (end effector pose when all joints = 0)
    M = np.array([
        [1, 0, 0, 0.5],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.7],
        [0, 0, 0, 1.0]
    ])
    
    # Screw axes for each joint (in space frame)
    Slist = np.array([
        [0, 0, 1,   0, 0, 0],       # Joint 1: rotation about Z
        [0, 1, 0,  -0.7, 0, 0],     # Joint 2
        [0, 1, 0,  -0.7, 0, 0.2],   # Joint 3
        [0, 0, 1,   0, 0, 0.2],     # Joint 4
        [0, 1, 0,  -0.7, 0, 0.5],   # Joint 5
        [0, 0, 1,   0, 0, 0.5],     # Joint 6
        [0, 1, 0,  -0.7, 0, 0.7]    # Joint 7
    ]).T
    
    # Compute end effector pose
    T = mr.FKinSpace(M, Slist, joint_angles)
    return T

# Example
joints = np.array([0, 0.5, 0, 0.5, 0, 0.5, 0])
T_ee = forward_kinematics_arm(joints)
print(f"End effector position: {T_ee[:3, 3]}")
```

## Inverse Kinematics

### Numerical IK with modern_robotics

```python
def inverse_kinematics_arm(target_pose, initial_guess, eomg=0.001, ev=0.001):
    """
    Solve IK numerically using Newton-Raphson method
    
    Args:
        target_pose: 4x4 desired end effector pose
        initial_guess: initial joint angles
        eomg: angular error tolerance
        ev: linear error tolerance
    
    Returns:
        joint_angles: solution (or best attempt)
        success: whether IK converged
    """
    # Same M and Slist as in FK
    M, Slist = get_robot_model()
    
    joint_angles, success = mr.IKinSpace(
        Slist, M, target_pose, initial_guess, eomg, ev
    )
    
    return joint_angles, success

# Example: Reach to a point
target_pos = np.array([0.4, 0.2, 0.6])
target_pose = np.eye(4)
target_pose[:3, 3] = target_pos

initial_guess = np.zeros(7)
solution, success = inverse_kinematics_arm(target_pose, initial_guess)

if success:
    print(f"IK solution: {np.degrees(solution)}")
else:
    print("IK failed to converge")
```

### Jacobian-Based IK

```python
def jacobian_ik(target_pos, current_joints, max_iters=100, alpha=0.1):
    """
    Iterative IK using Jacobian pseudoinverse
    
    More efficient for real-time control
    """
    joints = current_joints.copy()
    
    for i in range(max_iters):
        # Current end effector position
        T = forward_kinematics_arm(joints)
        current_pos = T[:3, 3]
        
        # Position error
        error = target_pos - current_pos
        
        if np.linalg.norm(error) < 0.001:
            return joints, True
        
        # Compute Jacobian
        J = compute_jacobian(joints)  # 3x7 position Jacobian
        
        # Pseudoinverse
        J_pinv = np.linalg.pinv(J)
        
        # Update joints
        delta_joints = alpha * J_pinv @ error
        joints += delta_joints
    
    return joints, False
```

## Whole-Body Inverse Kinematics

```python
from scipy.optimize import minimize

class WholeBodyIK:
    def __init__(self, robot_model):
        self.robot = robot_model
        self.num_joints = 50  # Full humanoid
    
    def solve(self, tasks, current_joints):
        """
        Solve whole-body IK with multiple simultaneous tasks
        
        Args:
            tasks: List of Task objects (end effector goals, gaze targets, etc.)
            current_joints: Current joint configuration
        
        Returns:
            optimal_joints: Joint configuration satisfying all tasks (weighted)
        """
        
        def cost_function(joints):
            total_cost = 0
            
            # Task costs
            for task in tasks:
                task_cost = task.compute_error(self.robot, joints)
                total_cost += task.weight * task_cost**2
            
            # Regularization (prefer small joint velocities)
            joint_change = joints - current_joints
            total_cost += 0.01 * np.sum(joint_change**2)
            
            return total_cost
        
        # Constraints
        constraints = []
        
        # ZMP stability constraint
        constraints.append({
            'type': 'ineq',
            'fun': lambda j: self.check_zmp_stability(j)
        })
        
        # Joint limits
        bounds = [(self.robot.joint_limits[i][0], self.robot.joint_limits[i][1]) 
                  for i in range(self.num_joints)]
        
        # Optimize
        result = minimize(
            cost_function,
            current_joints,
            method='SLSQP',
            bounds=bounds,
            constraints=constraints
        )
        
        return result.x, result.success

# Example usage
class EndEffectorTask:
    def __init__(self, target_position, weight=1.0):
        self.target = target_position
        self.weight = weight
    
    def compute_error(self, robot, joints):
        T = robot.forward_kinematics(joints)
        current_pos = T[:3, 3]
        return np.linalg.norm(current_pos - self.target)

class BalanceTask:
    def __init__(self, weight=10.0):
        self.weight = weight
    
    def compute_error(self, robot, joints):
        com = robot.compute_com(joints)
        support_center = robot.get_support_polygon_center(joints)
        return np.linalg.norm(com[:2] - support_center)

# Solve
ik_solver = WholeBodyIK(robot_model)
tasks = [
    EndEffectorTask(target_position=[0.4, 0.3, 0.8], weight=1.0),
    BalanceTask(weight=10.0)
]
solution, success = ik_solver.solve(tasks, current_joints)
```

## MoveIt2 Integration

### Configuration

```yaml
# config/moveit_config.yaml
planning_groups:
  - name: left_arm
    joints:
      - left_shoulder_pan
      - left_shoulder_lift
      - left_elbow
      - left_wrist_1
      - left_wrist_2
      - left_wrist_3
      - left_wrist_4
    
  - name: right_arm
    joints:
      - right_shoulder_pan
      # ...
```

### Using MoveIt2 in Code

```python
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

class MoveItPlanner(Node):
    def __init__(self):
        super().__init__('moveit_planner')
        
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            'move_action'
        )
    
    def plan_to_pose(self, target_pose, group_name="left_arm"):
        """Plan motion to target pose using MoveIt2"""
        
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        
        # Set target pose
        goal.request.goal_constraints.append(
            self.create_pose_constraint(target_pose)
        )
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()
```

## Lab Exercises

### Lab 12.1: IK Solver Comparison
Compare numerical IK (Newton-Raphson) vs. Jacobian pseudoinverse for speed and accuracy.

### Lab 12.2: Whole-Body Controller
Implement controller that simultaneously reaches with right hand while maintaining balance.

### Lab 12.3: MoveIt2 Setup
Configure MoveIt2 for full humanoid, plan bimanual pick-and-place.

## Summary

✅ **Forward kinematics** maps joint angles to end effector poses  
✅ **Inverse kinematics** finds joint angles for desired poses (multiple solutions possible)  
✅ **Whole-body IK** coordinates all DOF with multiple task priorities  
✅ **MoveIt2** provides collision-aware motion planning

---

**Next**: [Speech-to-Action with Whisper and GPT](../week13-conversational-robotics/speech-to-action-whisper-gpt)
