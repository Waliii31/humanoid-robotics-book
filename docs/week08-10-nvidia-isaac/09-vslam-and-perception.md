---
sidebar_position: 9
title: vSLAM and Perception
---

# Visual SLAM and Perception in Isaac Sim

## Learning Objectives

- Implement Visual SLAM (vSLAM) algorithms in Isaac Sim
- Use Isaac Sim's perception libraries for object detection
- Generate synthetic datasets with ground truth labels
- Apply domain randomization for sim-to-real transfer
- Integrate computer vision pipelines with ROS 2

## Introduction

Visual SLAM enables robots to build maps and localize simultaneously using camera data. Isaac Sim's synthetic data generation accelerates perception AI development by creating large labeled datasets automatically.

## Visual SLAM with RGB-D Camera

### Isaac Sim Camera Setup

```python
from omni.isaac.sensor import Camera
from omni.isaac.core.utils import prims

# Create RGB-D camera
camera_prim = prims.create_prim(
    "/World/Camera",
    "Camera",
    position=np.array([2.0, 0.0, 1.5]),
    orientation=np.array([0, 0, 0, 1])
)

camera = Camera(
    prim_path="/World/Camera",
    frequency=30,
    resolution=(1280, 720)
)

# Enable depth output
camera.add_distance_to_image_plane()
```

### RTAB-Map Integration

```bash
# Install RTAB-Map
sudo apt install ros-humble-rtabmap-ros

# Launch with Isaac Sim camera topics
ros2 launch rtabmap_ros rtabmap.launch.py \
    rgb_topic:=/camera/rgb/image_raw \
    depth_topic:=/camera/depth/image_raw \
    camera_info_topic:=/camera/rgb/camera_info
```

## Synthetic Data Generation

### Replicator API

```python
import omni.replicator.core as rep

# Setup render products
camera = rep.create.camera(position=(2, 0, 1.5))
render_product = rep.create.render_product(camera, resolution=(1024, 1024))

# Randomization
def randomize_scene():
    # Random object poses
    objects = rep.get.prims(path_pattern="/World/Objects/*")
    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform((-1, -1, 0.5), (1, 1, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )
    
    # Random lighting
    lights = rep.get.prims(semantics=[('class', 'light')])
    with lights:
        rep.modify.attribute("intensity", rep.distribution.uniform(1000, 5000))

# Register randomization
rep.randomizer.register(randomize_scene)

# Capture annotated data
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="./output", rgb=True, bounding_box_2d_tight=True)

with camera:
    rep.trigger.on_frame(num_frames=1000, rt_subframes=5)

rep.orchestrator.run()
```

## Perception Pipeline

### YOLOv8 Integration

```python
from ultralytics import YOLO
import cv2

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.model = YOLO('yolov8n.pt')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        
        # Run detection
        results = self.model(image)
        
        # Publish detections
        detections = Detection2DArray()
        for result in results[0].boxes:
            detection = Detection2D()
            detection.bbox.center.x = result.xywh[0][0]
            detection.bbox.center.y = result.xywh[0][1]
            detection.bbox.size_x = result.xywh[0][2]
            detection.bbox.size_y = result.xywh[0][3]
            detections.detections.append(detection)
        
        self.detection_pub.publish(detections)
```

## Domain Randomization

```python
# Randomize textures, lighting, camera params
def domain_randomization():
    # Vary object materials
    materials = rep.get.prims(semantics=[('class', 'material')])
    with materials:
        rep.randomizer.color(colors=rep.distribution.uniform((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)))
    
    # Vary lighting
    rep.modify.attribute("intensity", rep.distribution.normal(2000, 500))
    
    # Vary camera parameters
    camera_prim = rep.get.prims(path_pattern="/World/Camera")
    with camera_prim:
        rep.modify.attribute("focalLength", rep.distribution.uniform(18, 35))
```

## Lab Exercises

### Lab 9.1: vSLAM Dataset
Generate 10,000 RGB-D images with ground truth poses for SLAM evaluation.

### Lab 9.2: Custom Object Detection
Train YOLOv8 on synthetic data, test on real-world images.

## Summary

✅ **vSLAM** builds maps and localizes using camera data  
✅ **Synthetic data** accelerates perception AI training  
✅ **Domain randomization** improves sim-to-real transfer  
✅ **Ground truth labels** enable supervised learning

---

**Next**: [Advanced Isaac Features](./advanced-isaac-features)
