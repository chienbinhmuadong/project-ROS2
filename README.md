# ROS2 Diff Drive Robot – Obstacle Avoidance System

## Introduction

This project implements a Differential Drive robot control system using **ROS2 Control** combined with **Camera** and **LiDAR** sensors to perform intelligent obstacle avoidance.

The robot is capable of:

- Controlling motion using `ros2_control`
- Detecting white holes on the ground using a camera
- Measuring front obstacles using LiDAR
- Making navigation decisions based on sensor data

---

## Technologies Used

- ROS2
- ros2_control
- diff_drive_controller
- LiDAR sensor
- Camera sensor (image processing)
- Gazebo (Simulation)
- RViz2

---

## System Architecture

### 1️⃣ Motion Control

- Managed by `ros2_control`
- Uses `diff_drive_controller` for wheel velocity control

### White Hole Detection (Camera-Based)

- Camera processes image data in real time
- Detects abnormal white regions on the ground
- If a white hole is detected:
  - Robot stops
  - Or changes direction to avoid falling

### Obstacle Detection (LiDAR-Based)

- LiDAR publishes data on `/scan`
- Distance threshold is defined for safe navigation
- If obstacle distance < safety threshold:
  - Robot rotates to avoid collision

---


## Future Improvements

- Improve image processing algorithm
- Integrate SLAM for localization
- Optimize obstacle avoidance logic
- Deploy on real robot hardware

---


## ▶️ Build & Run

```bash
colcon build
source install/setup.bash
ros2 launch ros_project_scene bringup.launch.py
