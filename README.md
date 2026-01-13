## Calibration with arUco marker and Realsense camera for ros1
This github provides a simple tutorial for eye-on-base camera calibration with arUco marker and Realsense camera setting for franka robot in ros1

### Getstart
#### 1. Install aruco-ros, realsense-ros, ddynamic_reconfigure

```bash
cd catkin_ws/src
git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros.git
git clone -b ros1-legacy https://github.com/realsenseai/realsense-ros.git
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
cd ..
catkin build



