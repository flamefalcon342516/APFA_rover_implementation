# APFA_Rover_Implementation

This repository contains the implementation of the **Artificial Potential Field Algorithm (APFA)** for autonomous navigation of a Pixhawk-based rover using real-world point cloud data from a **depth camera**. The rover computes attractive and repulsive forces to navigate towards a goal while avoiding obstacles in its environment.

---

## 🚀 System Overview

- **Depth Camera** (Intel RealSense D455) publishes 3D point cloud data.
- **ROS 2 Node** computes attractive and repulsive forces using APFA.
- **DroneKit** sends velocity commands to the **Pixhawk** flight controller.
- **RViz2** and command-line tools are used for monitoring and debugging.

---

## 📦 Setup Instructions

### 1. **Launch the Depth Camera**

First, connect the Intel RealSense D455 camera to your laptop and launch the node:

```bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
