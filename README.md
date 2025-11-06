# Object Triggered SLAM

This project implements a **ROS 2-based SLAM system triggered by object detection** using 2D LiDAR data.  
When an object (e.g. cone-like shape) is detected from LiDAR clustering, the robot automatically navigates to the object's location and continues mapping.  

This approach reduces unnecessary exploration and focuses on mapping object-relevant regions.

<p align="center">
  <img src="docs/demo_screenshot.png" width="600"/>
</p>

<p align="center">
  <a href="https://youtu.be/your_video_link"><img src="docs/demo_thumbnail.gif" width="600"/></a><br>
  <em>▶️ Click to watch the demo video</em>
</p>

---

## 📦 Features

- Object-based exploration trigger using 2D LiDAR
- Euclidean clustering for object candidate detection
- Integration with SLAM Toolbox (async mode)
- Navigation to target objects using Nav2
- Gazebo simulation with TurtleBot3 and custom four-wheel robot

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

---

## 🚀 How to Use

```bash
# Clone the repo
git clone https://github.com/TakiRyo/object-triggered-SLAM.git
cd object-triggered-SLAM/ros2_ws
colcon build --symlink-install
source install/setup.bash
