## Calibration with arUco marker and Realsense camera for ros1
This github provides a simple tutorial for eye-on-base camera calibration with arUco marker and Realsense camera setting for franka robot in ros1

### Getstart
#### 1. Install aruco-ros, realsense-ros, ddynamic_reconfigure

```bash
cd catkin_ws/src # replace with path to your workspace

git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros.git
git clone -b ros1-legacy https://github.com/realsenseai/realsense-ros.git
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git

cd ..
catkin build
```

#### 2. Install easy_handeye

```bash
cd src # replace with path to your workspace
git clone https://github.com/IFL-CAMP/easy_handeye

cd..
rosdep install -iyr --from-paths src

catkin build
```

#### 3. modify panda_realsense_eyeonbase.launch with your own setting

modify the below args to fit your own setting
```xml
<launch>
    <arg name="namespace_prefix" default="panda_eob_calib"/>

    <include file="$(find panda_moveit_config)/launch/franka_control.launch">
        <!-- set your robot ip -->
        <arg name="robot_ip" value="franka4"/> 
        <arg name="load_gripper" value="true"/>
    </include>        
    
    <include file="$(find realsense2_camera)/launch/rs_camera.launch">
        <!-- set your camera parameters -->
        <arg name="color_width"  value="1280"/>
        <arg name="color_height" value="720"/>
        <arg name="color_fps"    value="30"/>
        <arg name="depth_width" value="1280"/>
        <arg name="depth_height" value="720"/>
        <arg name="depth_fps"   value="30"/>
    </include>

        <arg name="markerId"        default="26"/>      <!-- set your marker id -->
        <arg name="markerSize"      default="0.05"/>    <!-- set your marker size, in m -->
        <arg name="eye"             default="left"/>
        <arg name="marker_frame"    default="aruco_marker_frame"/> <!-- set your maker frame -->
        <arg name="ref_frame"       default=""/>  <!-- leave empty and the pose will be published wrt param parent_name -->
        <arg name="corner_refinement" default="LINES" /> <!-- NONE, HARRIS, LINES, SUBPIX -->    
        <arg name="camera_frame" default="camera_color_frame" /> <!-- set your camera frame -->
        <arg name="camera_image_topic" default="/camera/color/image_raw" /> <!-- set your image topic -->
        <arg name="camera_info_topic" default="/camera/color/camera_info" /> <!-- set your camera info topic -->

    <node pkg="aruco_ros" type="single" name="aruco_single">
        <remap to="$(arg camera_info_topic)" from="/camera_info" />
        <remap to="$(arg camera_image_topic)" from="/image" />
        <param name="image_is_rectified" value="True"/>
        <param name="marker_size"        value="$(arg markerSize)"/>
        <param name="marker_id"          value="$(arg markerId)"/>
        <param name="reference_frame"    value="$(arg ref_frame)"/>   
        <param name="camera_frame"       value="$(arg camera_frame)"/>
        <param name="marker_frame"       value="$(arg marker_frame)" />
        <param name="corner_refinement"  value="$(arg corner_refinement)" />
    </node>


    <!-- (start hand-eye-calibration) -->
    <include file="$(find easy_handeye)/launch/calibrate.launch">
        <arg name="eye_on_hand" value="false"/>
        <arg name="namespace_prefix" value="$(arg namespace_prefix)"/>
        <arg name="move_group" value="panda_manipulator"  doc="the name of move_group for the automatic robot motion with MoveIt!" />
        <arg name="freehand_robot_movement" value="false"/>
        <arg name="publish_dummy" value="false"/> <!-- set to be false -->

        <!-- fill in the following parameters according to your robot's published tf frames -->
        <arg name="robot_base_frame" value="panda_link0"/>
        <arg name="robot_effector_frame" value="panda_hand_tcp"/>

        <!-- fill in the following parameters according to your tracking system's published tf frames -->
        <arg name="tracking_base_frame" value="camera_color_optical_frame"/> <!-- set your tracking base frame -->
        <arg name="tracking_marker_frame" value="aruco_marker_frame"/> <!-- set your tracking marker frame-->
    </include>


</launch>
```
After modifying the launch file, save it under ```catkin_ws/src/easy_handeye/easy_handeye/launch/```

#### 4. Bridge arUco pose topic into TF
download ```transform_to_tf.py``` to ```catkin_ws/src/script/```(can be any directory runable with python3), in one terminal, run:
```bash
python3 src/script/transform_to_tf.py
```




