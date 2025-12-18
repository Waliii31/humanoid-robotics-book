# 13-Week Humanoid Robotics Curriculum Specification

## Course Information

**Course Title**: Humanoid Robotics: From Embodied AI to Conversational Agents  
**Duration**: 13 Weeks  
**Level**: Advanced Undergraduate / Graduate  
**Prerequisites**: Python programming, basic linear algebra, Linux/Ubuntu familiarity  
**Contact Hours**: 3 hours lecture + 3 hours lab per week  

---

## Learning Outcomes

By the end of this 13-week course, students will be able to:

1. **Understand and apply** embodied AI principles to humanoid robotics
2. **Design and implement** ROS 2 systems for robot control and communication
3. **Create and simulate** humanoid robots using Gazebo and URDF
4. **Utilize** NVIDIA Isaac Sim for advanced perception and simulation
5. **Develop** bipedal locomotion systems using kinematics and dynamics
6. **Integrate** conversational AI (Whisper, GPT) with physical robot actions
7. **Deploy** complete humanoid robot systems from simulation to reality

---

## Weekly Breakdown

### **Weeks 1-2: Introduction to Physical AI and Embodied Intelligence**

#### Week 1: Foundations of Physical AI
**File**: `docs/week01-02-physical-ai/01-foundations-of-physical-ai.md`

**Learning Objectives**:
- Define Physical AI and embodied intelligence
- Differentiate between disembodied and embodied AI
- Understand the perception-action loop
- Identify applications of Physical AI in humanoid robotics

**Topics**:
- What is Physical AI?
- Embodied cognition paradigm
- Perception-action-learning cycle
- History of humanoid robotics (ASIMO, Atlas, Optimus)
- Current state-of-the-art

**Lab**:
- Survey of humanoid robots (video analysis)
- Install Ubuntu 22.04 and basic tools

#### Week 2: Embodied Intelligence Architecture
**File**: `docs/week01-02-physical-ai/02-embodied-intelligence-architecture.md`

**Learning Objectives**:
- Understand sensor-motor integration
- Explain whole-body control concepts
- Identify key subsystems in humanoid robots
- Design high-level robot architectures

**Topics**:
- Sensor systems (exteroceptive, proprioceptive)
- Actuator types and selection
- Control hierarchies
- Software architecture patterns
- Real-time constraints

**Lab**:
- Analyze humanoid robot specs (Boston Dynamics, Tesla)
- Design a conceptual humanoid architecture

---

### **Weeks 3-5: ROS 2 Fundamentals**

#### Week 3: ROS 2 Ecosystem and Nodes
**File**: `docs/week03-05-ros2-fundamentals/03-ros2-ecosystem-and-nodes.md`

**Learning Objectives**:
- Install and configure ROS 2 Humble
- Understand DDS middleware
- Create basic ROS 2 nodes in Python and C++
- Use ROS 2 command-line tools

**Topics**:
- ROS 2 vs ROS 1
- DDS and QoS concepts
- Node lifecycle
- Package structure (ament_python, ament_cmake)
- rqt tools

**Lab**:
- Install ROS 2 Humble
- Create first publisher/subscriber nodes
- Build and run ROS 2 packages

#### Week 4: Topics, Services, and Actions
**File**: `docs/week03-05-ros2-fundamentals/04-topics-services-actions.md`

**Learning Objectives**:
- Implement publish-subscribe communication
- Create service servers and clients
- Use action servers for long-running tasks
- Select appropriate communication patterns

**Topics**:
- Topic communication patterns
- Service request-reply pattern
- Action goals, feedback, and results
- Message types (std_msgs, geometry_msgs, sensor_msgs)
- Custom message creation

**Lab**:
- Build multi-node robot controller
- Implement service-based configuration
- Create action server for trajectory execution

#### Week 5: Parameters, Launch Files, and TF2
**File**: `docs/week03-05-ros2-fundamentals/05-parameters-launch-tf2.md`

**Learning Objectives**:
- Manage robot parameters dynamically
- Create complex launch files
- Understand and use TF2 for coordinate transforms
- Visualize robot state in RViz2

**Topics**:
- Parameter declaration and callbacks
- Launch file composition
- TF2 transforms and frames
- URDF and robot_state_publisher
- RViz2 configuration

**Lab**:
- Create parameterized robot controller
- Build launch files for multi-node systems
- Set up TF tree for simple robot

---

### **Weeks 6-7: Robot Simulation with Gazebo and URDF**

#### Week 6: URDF Modeling and Gazebo Basics
**File**: `docs/week06-07-simulation/06-urdf-modeling-gazebo-basics.md`

**Learning Objectives**:
- Write URDF descriptions for robots
- Understand links, joints, and kinematic chains
- Launch robots in Gazebo Classic
- Configure physics and sensor plugins

**Topics**:
- URDF syntax (links, joints, meshes)
- Xacro for modular URDF
- Gazebo world files
- Gazebo plugins (camera, IMU, contact sensors)
- Physics engines (ODE, Bullet, Simbody)

**Lab**:
- Create URDF model of 7-DOF robot arm
- Spawn robot in Gazebo
- Add sensors (camera, IMU) to robot

#### Week 7: Gazebo-ROS Integration and Control
**File**: `docs/week06-07-simulation/07-gazebo-ros-integration-control.md`

**Learning Objectives**:
- Integrate Gazebo with ROS 2
- Implement ROS 2 control framework
- Use joint trajectory controllers
- Simulate sensor data

**Topics**:
- gazebo_ros_pkgs
- ros2_control framework
- JointTrajectoryController
- DiffDriveController
- Sensor data publishing from Gazebo

**Lab**:
- Implement ROS 2 control for robot arm
- Create teleoperation system
- Simulate camera and depth sensors

---

### **Weeks 8-10: NVIDIA Isaac Platform**

#### Week 8: Isaac Sim Introduction and Setup
**File**: `docs/week08-10-isaac-platform/08-isaac-sim-introduction.md`

**Learning Objectives**:
- Install and configure Isaac Sim
- Understand Omniverse platform
- Import and manipulate 3D assets
- Run basic Isaac Sim simulations

**Topics**:
- Isaac Sim vs Gazebo comparison
- Omniverse USD format
- Photorealistic rendering
- Physics simulation (PhysX)
- ROS 2 bridge setup

**Lab**:
- Install Isaac Sim
- Import robot model from URDF
- Create simple simulation scene
- Test ROS 2 communication

#### Week 9: Visual SLAM and Perception in Isaac Sim
**File**: `docs/week08-10-isaac-platform/09-vslam-and-perception.md`

**Learning Objectives**:
- Implement Visual SLAM (vSLAM) in Isaac Sim
- Use Isaac Sim's perception libraries
- Generate synthetic training data
- Integrate computer vision pipelines

**Topics**:
- Visual SLAM algorithms (ORB-SLAM, RTAB-Map)
- Isaac Sim synthetic data generation
- Ground truth pose and depth
- Domain randomization
- Replicator for dataset creation

**Lab**:
- Implement vSLAM with RGB-D camera
- Create synthetic dataset (10K images)
- Compare real vs synthetic data

#### Week 10: Advanced Isaac Sim Features
**File**: `docs/week08-10-isaac-platform/10-advanced-isaac-features.md`

**Learning Objectives**:
- Use Isaac Gym for reinforcement learning
- Implement multi-robot scenarios
- Optimize simulation performance
- Deploy trained policies

**Topics**:
- Isaac Gym tensor API
- Parallel simulation instances
- RL policy training (PPO, SAC)
- Sim-to-real transfer techniques
- Performance profiling

**Lab**:
- Train RL policy for object manipulation
- Run 100 parallel simulation instances
- Evaluate policy generalization

---

### **Weeks 11-12: Humanoid Development**

#### Week 11: Bipedal Locomotion Fundamentals
**File**: `docs/week11-12-humanoid-dev/11-bipedal-locomotion-fundamentals.md`

**Learning Objectives**:
- Understand bipedal walking mechanics
- Implement Zero Moment Point (ZMP) criterion
- Calculate center of mass trajectories
- Design simple walking gaits

**Topics**:
- Bipedal stability (ZMP, COP)
- Gait cycle phases
- Inverted pendulum model
- Footstep planning
- Balance control

**Lab**:
- Calculate ZMP for walking sequence
- Implement inverted pendulum controller
- Simulate 2D biped in Gazebo

#### Week 12: Kinematics and Whole-Body Control
**File**: `docs/week11-12-humanoid-dev/12-kinematics-whole-body-control.md`

**Learning Objectives**:
- Solve forward and inverse kinematics
- Implement whole-body IK
- Use MoveIt2 for humanoid planning
- Coordinate multiple task objectives

**Topics**:
- Forward kinematics (DH parameters, PoE)
- Numerical inverse kinematics
- Jacobian and differential kinematics
- Whole-body optimization
- MoveIt2 motion planning

**Lab**:
- Implement IK solver for 7-DOF arm
- Configure MoveIt2 for full humanoid
- Plan and execute whole-body motions

---

### **Week 13: Conversational Robotics**

#### Week 13: Speech-to-Action with Whisper and GPT
**File**: `docs/week13-conversational/13-speech-to-action-whisper-gpt.md`

**Learning Objectives**:
- Integrate OpenAI Whisper for speech recognition
- Use GPT for natural language understanding
- Map language to robot actions
- Build end-to-end conversational robot system

**Topics**:
- Speech recognition with Whisper
- Natural language understanding (GPT-4/3.5)
- Intent classification and slot filling
- Action grounding (language → motion)
- Error handling and clarification
- Multi-turn dialogue management

**Lab**:
- Implement Whisper ROS 2 node
- Create GPT-based action planner
- Build voice-controlled humanoid demo
- Integration: "Pick up the red block" → execution

---

## Assessment Structure

| Assessment Type | Weight | Description |
|----------------|--------|-------------|
| **Weekly Labs** | 30% | Hands-on coding assignments (13 total) |
| **Midterm Project** | 25% | Weeks 1-7 integration (ROS 2 + Gazebo robot) |
| **Final Project** | 35% | Complete humanoid with conversational interface |
| **Participation** | 10% | Class engagement, peer code reviews |

### Midterm Project (Week 7)
**Deliverable**: Simulated robot in Gazebo with:
- Custom URDF model
- ROS 2 control interface
- Sensor integration (camera, IMU)
- Teleoperation capability

### Final Project (Week 13)
**Deliverable**: Humanoid robot system with:
- Isaac Sim simulation
- Bipedal walking capability
- Voice command interface (Whisper + GPT)
- Object manipulation
- 5-minute live demo + documentation

---

## Required Software Stack

### Core Tools
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Classic 11 or Gazebo Fortress
- Python 3.10+
- C++ (gcc 11+)

### NVIDIA Stack (GPU Required)
- NVIDIA Isaac Sim 2023.1.1+
- CUDA 11.8+
- cuDNN 8.6+
- Isaac Gym (optional for RL)

### AI/ML Libraries
- OpenAI API (Whisper, GPT)
- PyTorch 2.0+
- transformers (Hugging Face)
- opencv-python
- NumPy, SciPy, Matplotlib

### Robotics Libraries
- MoveIt2
- Navigation2
- ros2_control
- modern_robotics (Python)

---

## Hardware Recommendations

### Minimum (Weeks 1-7)
- CPU: 4 cores, 2.5 GHz
- RAM: 8 GB
- Storage: 50 GB
- GPU: Integrated graphics

### Recommended (Full Course)
- CPU: 8 cores, 3.0 GHz
- RAM: 16 GB
- Storage: 100 GB SSD
- GPU: NVIDIA RTX 3060 (6 GB VRAM)

### Optimal (Advanced Projects)
- CPU: 12+ cores, 3.5 GHz
- RAM: 32 GB
- Storage: 500 GB NVMe SSD
- GPU: NVIDIA RTX 4070 (12 GB VRAM)

---

## Chapter Structure Template

Each chapter must include:

```markdown
---
sidebar_position: [week_number]
title: [Chapter Title]
---

# [Chapter Title]

## Learning Objectives
[3-5 specific, measurable objectives]

## Introduction
[Engaging introduction, 2-3 paragraphs]

## Theoretical Background
[Core concepts with equations if needed]

## Practical Implementation
[Code examples, minimum 2 blocks]

## Hands-On Lab
[Step-by-step guided exercise]

## Challenge Exercise
[Open-ended problem for advanced students]

## Summary
[Key takeaways, 5-7 bullet points]

## Further Reading
[3-5 resources: papers, books, tutorials]

## Quiz Questions (Self-Assessment)
[5 multiple choice or short answer]
```

---

## Folder Structure

```
docs/
├── week01-02-physical-ai/
│   ├── 01-foundations-of-physical-ai.md
│   └── 02-embodied-intelligence-architecture.md
├── week03-05-ros2-fundamentals/
│   ├── 03-ros2-ecosystem-and-nodes.md
│   ├── 04-topics-services-actions.md
│   └── 05-parameters-launch-tf2.md
├── week06-07-simulation/
│   ├── 06-urdf-modeling-gazebo-basics.md
│   └── 07-gazebo-ros-integration-control.md
├── week08-10-isaac-platform/
│   ├── 08-isaac-sim-introduction.md
│   ├── 09-vslam-and-perception.md
│   └── 10-advanced-isaac-features.md
├── week11-12-humanoid-dev/
│   ├── 11-bipedal-locomotion-fundamentals.md
│   └── 12-kinematics-whole-body-control.md
└── week13-conversational/
    └── 13-speech-to-action-whisper-gpt.md
```

---

## Grading Rubric for Labs

| Criteria | Excellent (90-100%) | Good (75-89%) | Satisfactory (60-74%) | Needs Work (<60%) |
|----------|-------------------|---------------|---------------------|------------------|
| **Functionality** | All requirements met, no bugs | Minor issues, mostly working | Core functionality present | Significant issues |
| **Code Quality** | Clean, documented, follows style | Generally good structure | Some organizational issues | Poor structure |
| **Innovation** | Goes beyond requirements | Meets all requirements | Meets minimum requirements | Below requirements |
| **Documentation** | Comprehensive, clear | Good explanations | Basic documentation | Minimal/unclear |

---

## References & Resources

### Textbooks
1. "Modern Robotics" by Lynch & Park
2. "Probabilistic Robotics" by Thrun, Burgard, Fox
3. "Humanoid Robotics: A Reference" by Goswami & Vadakkepat

### Online Courses
1. ROS 2 Documentation: https://docs.ros.org/en/humble/
2. NVIDIA Isaac Sim Tutorials: https://docs.omniverse.nvidia.com/isaacsim/
3. Gazebo Tutorials: https://gazebosim.org/docs

### Papers
1. RT-2: Vision-Language-Action Models (Google DeepMind, 2023)
2. Atlas: The Agile Anthropomorphic Robot (Boston Dynamics)
3. Optimus: Humanoid Robot Development (Tesla AI)

---

**Document Version**: 1.0  
**Last Updated**: 2025-12-17  
**Author**: [Course Instructor Name]  
**License**: Educational Use
