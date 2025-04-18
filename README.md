# APFA_Rover_Implementation

This repository contains the implementation of the **Artificial Potential Field Algorithm (APFA)** for autonomous navigation of a Pixhawk-based rover using real-world point cloud data from a **depth camera**. The rover computes attractive and repulsive forces to navigate towards a goal while avoiding obstacles in its environment

---
.![WhatsApp Image 2025-04-18 at 10 31 17 AM](https://github.com/user-attachments/assets/9b8765f8-145c-4376-8b01-4ebb9c3b3a19)

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
```
or alternatively use:

```bash
ros2 launch realsense2_camera rs_launch.py
```
### 2. **Verify Camera Output**
You can check if point cloud data is being published by:

Opening RViz2 and visualizing the /camera/depth/color/points topic.

Or running:

```bash
ros2 topic echo /camera/depth/color/points
```
### 3.**Connect to Pixhawk**
Connect your laptop to the Pixhawk via USB and launch MAVProxy:

```bash
mavproxy.py --map --console
```
Once MAVProxy starts, verify connection to the Pixhawk and set the mode:
```bah
mode guided
 ```
🔐 Safety Tip: Always connect the Pixhawk to your RC receiver and keep your RC controller ready. In case the rover behaves unexpectedly, immediately switch the mode to Hold or take manual control via the RC.

### 4. **Run the APFA Navigation Code**
With the camera and Pixhawk set up, you can now launch the APFA-based navigation script:
```bash
python3 maze_runner.py
```
You may also use an alternative script ([new_apfa_dbscan.py]), but maze_runner.py is recommended for more stable performance.

![WhatsApp Image 2025-04-18 at 10 29 54 AM](https://github.com/user-attachments/assets/ff128e7c-db9c-4ddf-8883-dcc09375bb46)

