# Inherited-Octree-Map
This is a C++ project, which can build a octree map with the inherited connection relationship in **real-time**.

The project was developed and tested on **Ubuntu16.04** with **ROS Kinetic**. Some modifications may required in other platforms.

# Requirement
 - ROS Kinetic: http://wiki.ros.org/kinetic/Installation
 - Pangolin: https://github.com/stevenlovegrove/Pangolin. Download and install it.
 - Vins-Fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion. You need clone this VIO project to generate points cloud. Of course, other method can also be used with some modifications of this source code.
 - Kitti Visual Odometry: http://www.cvlibs.net/datasets/kitti/eval_odometry.php. Download it for Vins-Fusion. Please download: **odometry data set (grayscale, 22 GB)**


# Preview
You may view the results in "Demo.jpg".

Video: https://www.bilibili.com/video/BV1vv41117kE

# Quick Start
 - Run roscore
 ```
 rosrun roscore
 ```
 - Run Vins-Fusion Setero with kitti odom.
 ```
 source devel/setup.bash
 rosrun vins kitt_odom_test ~/vins/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml /home/data/00
 ```
 You may refer https://github.com/HKUST-Aerial-Robotics/VINS-Fusion for the usage.
 - Run inherted octree map.
 ```
 ./eight.sh
 ```

# Documentation
Sorry, this project has been abandoned. there is a only related documents of inherited quadtree map in Chinese.

https://blog.csdn.net/mkb9559/article/details/88050850
