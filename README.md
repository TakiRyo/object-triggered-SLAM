# Object Triggered SLAM

This project implements a **ROS 2-based SLAM system triggered by object detection** using 2D LiDAR data.  
When the robot detects objects (e.g., table-like clusters) via LiDAR clustering, it navigates toward them and continues mapping — enabling **efficient, object-driven exploration**.

>  **Current Status**:  
> - Implemented **2D LiDAR-based object detection**
> - Using **TurtleBot3 Waffle Pi** as the base robot in Gazebo simulation
> - Preparing a **demo video** to showcase functionality

---

## 💡 Motivation

In real-world robotics, blindly exploring every corner is inefficient.  
This project proposes a smarter method — **triggering exploration only when the robot detects something of interest.**

It could be useful for:
- Object-based search missions
- Warehouse robot tasks
- SLAM in environments with sparse but meaningful objects  

<p align="center">
  <img src="/docs/nav_pic.png" width="600"/>
</p>

▶️ [Click to watch the demo video](https://www.youtube.com/watch?v=AGsYb76OiyI)


---

## 📦 Features

- Object-based exploration trigger using 2D LiDAR
- Euclidean clustering for object candidate detection
- Integration with SLAM Toolbox (async mode)
- Navigation to target objects using Nav2
- Gazebo simulation with TurtleBot3 Waffle Pi

---

## 🔭 Future Work

- [ ] Add support for 3D LiDAR or RGB-D camera-based object detection
- [ ] Improve object classification (e.g., cone vs. wall)
- [ ] Evaluate SLAM quality quantitatively
- [ ] Apply to real-world TurtleBot3 with real sensors

---

## 🛠️ Pre-requirements

- **Ubuntu 22.04**
- **ROS 2 Humble**
- `slam_toolbox`
- `nav2_bringup`
- `turtlebot3_gazebo`, `turtlebot3_navigation2`, etc.
- RViz 2

> Make sure to source your ROS environment and set:
> ```bash
> export TURTLEBOT3_MODEL=waffle_pi
> ```
