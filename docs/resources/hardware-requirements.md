---
sidebar_position: 100
title: Hardware Requirements
---

# Hardware Requirements

## Overview

This page outlines the hardware requirements for the 13-Week Humanoid Robotics curriculum. Requirements vary significantly by module, from basic CPU-only setups for early weeks to NVIDIA RTX GPUs for advanced simulation and AI training.

:::tip Budget Planning
You can start the course with minimal hardware and upgrade as needed. Weeks 1-5 require only a standard laptop, while Weeks 8-13 benefit significantly from dedicated GPU hardware.
:::

---

## Requirements by Module

### Weeks 1-2: Physical AI & Embodied Intelligence

**Purpose**: Theory, concepts, reading, basic Python examples

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | Dual-core 2.0 GHz | Quad-core 2.5 GHz+ |
| **RAM** | 4 GB | 8 GB |
| **Storage** | 10 GB free | 20 GB SSD |
| **GPU** | Not required | Not required |
| **OS** | Windows 10/11, macOS, Linux | Ubuntu 22.04 LTS |

**Notes**: Standard laptop sufficient. Focus on reading and conceptual understanding.

---

### Weeks 3-5: ROS 2 Fundamentals

**Purpose**: ROS 2 installation, node creation, basic simulations

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | Quad-core 2.5 GHz | 6-core 3.0 GHz |
| **RAM** | 8 GB | 16 GB |
| **Storage** | 30 GB free | 50 GB SSD |
| **GPU** | Integrated graphics | NVIDIA GTX 1650 |
| **OS** | **Ubuntu 22.04 LTS** (required) | Ubuntu 22.04 LTS |

**Notes**:
- ROS 2 Humble officially supports Ubuntu 22.04
- Docker/WSL2 alternatives available for Windows/Mac but native Linux recommended
- GPU not required but helpful for RViz2 visualization

---

### Weeks 6-7: Simulation with Gazebo & URDF

**Purpose**: Gazebo Classic/Fortress, URDF modeling, physics simulation

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | 4 cores, 2.5 GHz | 8 cores, 3.0 GHz |
| **RAM** | 8 GB | 16 GB |
| **Storage** | 50 GB SSD | 100 GB SSD |
| **GPU** | NVIDIA GTX 1050 Ti (2GB VRAM) | NVIDIA RTX 3060 (6GB VRAM) |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

**Notes**:
- Gazebo benefits significantly from GPU acceleration
- Integrated graphics will work but with poor performance (5-10 FPS)
- NVIDIA GPUs recommended over AMD for better OpenGL support

---

### Weeks 8-10: NVIDIA Isaac Platform

**Purpose**: Isaac Sim, photorealistic rendering, synthetic data generation, RL training

| Component | Minimum | Recommended | Optimal |
|-----------|---------|-------------|---------|
| **CPU** | 6 cores, 3.0 GHz | 8 cores, 3.5 GHz | 12+ cores, 3.8 GHz |
| **RAM** | 16 GB | 32 GB | 64 GB |
| **Storage** | 100 GB SSD | 250 GB NVMe SSD | 500 GB NVMe SSD |
| **GPU** | **NVIDIA RTX 3060 (8GB VRAM)** | **NVIDIA RTX 4070 (12GB)** | **NVIDIA RTX 4090 (24GB)** |
| **CUDA** | 11.8+ | 12.0+ | 12.0+ |
| **Drivers** | Latest NVIDIA drivers | Latest NVIDIA drivers | Latest NVIDIA drivers |
| **OS** | Ubuntu 20.04/22.04 | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

**Critical Requirements**:
- ❗ **NVIDIA RTX GPU REQUIRED** (GTX series not supported)
- ❗ Minimum 8 GB VRAM (12+ GB recommended for complex scenes)
- ❗ Ray tracing cores beneficial for rendering
- ❗ Tensor cores beneficial for AI training

**Not Supported**:
- ❌ NVIDIA GTX series (lacks RTX features)
- ❌ AMD GPUs (no CUDA support)
- ❌ Intel integrated graphics
- ❌ Apple M1/M2 GPUs (no CUDA)

:::warning Isaac Sim Requirements
Isaac Sim is the most demanding component of this course. If you cannot meet the minimum RTX 3060 requirement, you can:
1. Use cloud computing (AWS, GCP with RTX instances)
2. Skip Isaac Sim modules and use Gazebo only
3. Partner with a classmate who has appropriate hardware
:::

---

### Weeks 11-12: Humanoid Development

**Purpose**: Bipedal locomotion, kinematics, whole-body control (in simulation)

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | 6 cores, 3.0 GHz | 8 cores, 3.5 GHz |
| **RAM** | 16 GB | 32 GB |
| **Storage** | 100 GB SSD | 250 GB SSD |
| **GPU** | NVIDIA RTX 3060 (8GB) | NVIDIA RTX 4070 (12GB) |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

**Notes**: Same as Isaac Sim requirements if using Isaac Gym for RL training.

---

### Week 13: Conversational Robotics

**Purpose**: Whisper speech recognition, GPT-4 integration, end-to-end system

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | Same as Weeks 11-12 | Same as Weeks 11-12 |
| **RAM** | 16 GB (32 GB if running Whisper locally) | 32 GB |
| **Storage** | 100 GB SSD | 250 GB SSD |
| **GPU** | RTX 3060 (8GB) | RTX 4070 (12GB) |
| **Internet** | Required for OpenAI API | High-speed recommended |

**Notes**:
- Whisper "base" model: ~1 GB VRAM, runs on CPU
- Whisper "large" model: ~10 GB VRAM, GPU recommended
- GPT-4 requires API key (cloud-based, minimal local resources)

---

## Recommended Hardware Configurations

### Budget Configuration (< $1,500)

**Target**: Complete course with acceptable performance

| Component | Specification | Est. Price |
|-----------|--------------|------------|
| **CPU** | AMD Ryzen 5 5600 (6-core) | $150 |
| **RAM** | 16 GB DDR4-3200 | $50 |
| **GPU** | NVIDIA RTX 3060 (12GB) | $350 |
| **Storage** | 500 GB NVMe SSD | $50 |
| **Motherboard** | B550 | $120 |
| **PSU** | 650W 80+ Bronze | $60 |
| **Case** | Mid-tower | $70 |
| **Total** | | **~$850** |

**Alternative: Used Workstation**
- Dell Precision 3650 with RTX 3060: ~$900-1200
- HP Z2 G8 with RTX 3060: ~$1000-1300

---

### Standard Configuration ($1,500 - $3,000)

**Target**: Smooth performance, some headroom for experimentation

| Component | Specification | Est. Price |
|-----------|--------------|------------|
| **CPU** | AMD Ryzen 7 5800X (8-core) | $250 |
| **RAM** | 32 GB DDR4-3600 | $100 |
| **GPU** | NVIDIA RTX 4070 (12GB) | $550 |
| **Storage** | 1 TB NVMe SSD | $80 |
| **Motherboard** | X570 | $180 |
| **PSU** | 750W 80+ Gold | $100 |
| **Case** | Mid-tower with good airflow | $100 |
| **Total** | | **~$1,360** |

---

### High-Performance Configuration ($3,000+)

**Target**: Professional-grade, research-level performance

| Component | Specification | Est. Price |
|-----------|--------------|------------|
| **CPU** | AMD Ryzen 9 7950X (16-core) or Intel i9-13900K | $500 |
| **RAM** | 64 GB DDR5-5600 | $250 |
| **GPU** | NVIDIA RTX 4090 (24GB) | $1,600 |
| **Storage** | 2 TB NVMe Gen4 SSD | $150 |
| **Motherboard** | X670E or Z790 | $350 |
| **PSU** | 1000W 80+ Platinum | $200 |
| **Case** | High-airflow tower | $150 |
| **Cooling** | AIO liquid cooler | $150 |
| **Total** | | **~$3,350** |

---

## Embedded Platforms (For Deployment)

### NVIDIA Jetson Orin Family

For deploying trained models on real robots:

#### Jetson Orin Nano

| Spec | Value |
|------|-------|
| **GPU** | 1024 CUDA cores, 32 Tensor cores |
| **CPU** | 6-core ARM Cortex-A78AE |
| **Memory** | 8 GB LPDDR5 |
| **AI Performance** | 40 TOPS (INT8) |
| **Power** | 7-15W |
| **Price** | $499 (Developer Kit) |

**Use Cases**:
- Small humanoid robots
- Prototype development
- Edge inference

#### Jetson Orin NX

| Spec | Value |
|------|-------|
| **GPU** | 1024 CUDA cores, 32 Tensor cores |
| **CPU** | 8-core ARM Cortex-A78AE |
| **Memory** | 16 GB LPDDR5 |
| **AI Performance** | 100 TOPS (INT8) |
| **Power** | 10-25W |
| **Price** | $599 (Module) |

**Use Cases**:
- Mid-size humanoids
- Multi-camera systems
- Real-time perception

#### Jetson AGX Orin

| Spec | Value |
|------|-------|
| **GPU** | 2048 CUDA cores, 64 Tensor cores |
| **CPU** | 12-core ARM Cortex-A78AE |
| **Memory** | 32 or 64 GB LPDDR5 |
| **AI Performance** | 275 TOPS (INT8) |
| **Power** | 15-60W |
| **Price** | $1,999 (32GB), $2,499 (64GB) |

**Use Cases**:
- Full-size humanoid robots
- Multi-modal AI (vision + language + action)
- Research platforms

---

## Cloud Computing Alternatives

If local hardware is unavailable or insufficient:

### AWS EC2 GPU Instances

| Instance Type | GPU | vCPUs | RAM | Price (On-Demand, us-east-1) |
|--------------|-----|-------|-----|------------------------------|
| **g5.xlarge** | A10G (24GB) | 4 | 16 GB | $1.006/hour |
| **g5.2xlarge** | A10G (24GB) | 8 | 32 GB | $1.212/hour |
| **p3.2xlarge** | V100 (16GB) | 8 | 61 GB | $3.06/hour |

**Cost Estimate** (Weeks 8-10):
- 3 weeks × 10 hours/week × $1.21/hour = **~$36**

### Google Cloud Platform

| Instance Type | GPU | vCPUs | RAM | Price (On-Demand) |
|--------------|-----|-------|-----|-------------------|
| **n1-standard-4 + T4** | T4 (16GB) | 4 | 15 GB | $0.50/hour |
| **n1-standard-8 + V100** | V100 (16GB) | 8 | 30 GB | $2.48/hour |

### Paperspace Gradient

| Machine Type | GPU | Price |
|-------------|-----|-------|
| **Free GPU** | M4000 (8GB) | Free (limited hours) |
| **P4000** | P4000 (8GB) | $0.51/hour |
| **RTX 4000** | RTX 4000 (8GB) | $0.56/hour |

---

## Laptop Recommendations

For students preferring portable solutions:

### Budget Laptop ($1,000 - $1,500)

- **Dell G15** with RTX 3050 Ti (4GB)
- **HP Victus 15** with RTX 3050 (4GB)
- **Lenovo Legion 5** with RTX 3060 (6GB)

:::warning Limited Performance
Budget laptop GPUs (RTX 3050/3050 Ti) have only 4GB VRAM, which may struggle with Isaac Sim. Consider cloud instances for Weeks 8-10.
:::

### Mid-Range Laptop ($1,500 - $2,500)

- **ASUS ROG Strix G16** with RTX 4060 (8GB)
- **MSI Raider GE68** with RTX 4070 (8GB)
- **Lenovo Legion Pro 7** with RTX 4080 (12GB)

### High-End Laptop ($2,500+)

- **Razer Blade 16** with RTX 4090 (16GB)
- **ASUS ROG Zephyrus Duo 16** with RTX 4090 (16GB)
- **MSI Titan GT77** with RTX 4090 (16GB)

---

## FAQ

### Can I use a Mac (M1/M2)?

**Partial support**:
- ✅ Weeks 1-2 (theory, Python)
- ⚠️ Weeks 3-5 (ROS 2 via Docker, slower)
- ❌ Weeks 6-10 (No CUDA, no Isaac Sim)
- ❌ Weeks 11-13 (Limited)

**Recommendation**: Use cloud instances for GPU-dependent modules.

### Can I use AMD GPUs?

**No** for this course. Critical tools require NVIDIA CUDA:
- Isaac Sim: NVIDIA only
- Isaac Gym: CUDA only
- Many ML frameworks optimize for CUDA

### What about integrated graphics?

Works for:
- ✅ Weeks 1-2
- ⚠️ Weeks 3-5 (slow RViz2)
- ❌ Weeks 6-13 (insufficient)

### Can I use Google Colab?

Limited usefulness:
- ROS 2: Difficult to set up
- Gazebo: No GUI support
- Isaac Sim: Not supported

**Better**: AWS/GCP/Paperspace with RDP/VNC access.

---

## Verification Commands

### Check CPU

```bash
lscpu | grep "Model name"
# Look for core count and clock speed
```

### Check RAM

```bash
free -h
# Total should show available RAM
```

### Check GPU

```bash
nvidia-smi
# Shows GPU model, VRAM, CUDA version
```

### Check CUDA

```bash
nvcc --version
# Should show CUDA 11.8 or newer
```

### Check Disk Space

```bash
df -h
# Check available space on /
```

---

## Summary

| Module | Minimum GPU | Recommended GPU | Can Use Cloud? |
|--------|-------------|-----------------|----------------|
| Weeks 1-2 | None | None | N/A |
| Weeks 3-5 | Integrated | GTX 1650 | Not practical |
| Weeks 6-7 | GTX 1050 Ti | RTX 3060 | Yes |
| Weeks 8-10 | **RTX 3060** | RTX 4070 | **Yes (recommended)** |
| Weeks 11-12 | RTX 3060 | RTX 4070 | Yes |
| Week 13 | RTX 3060 | RTX 4070 | Yes |

**Our Recommendation**: 
- **Local Hardware**: RTX 3060 12GB minimum (budget: ~$1,000-1,500 build)
- **Cloud Hybrid**: Basic laptop + cloud GPU for Weeks 8-13 (cost: ~$100-200)

---

**Last Updated**: 2025-12-17  
**Questions?** Contact course instructor
