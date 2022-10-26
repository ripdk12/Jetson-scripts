# Jetson-scripts
Convenience scripts and Dockerfiles for Jetson Nano

## Useful Links

#### Guide for flashing and booting from ssd/usb device
https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3261/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html#wwpID0E0NK0HA

#### Solution to librealsense2 certificate issue
https://github.com/IntelRealSense/librealsense/issues/10980

#### gg

I have setup the docker development environment with isaac_ros_visual_slam and RealSense D435i camera on a Xavier NX (JP5.0.2), following the guide on github, however, the results are extremely poor as shown in the picture. 
I believe it's similar to the issue described here: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/issues/54


I am mostly interested in a free solution for visual inertial odometry, (currently using slamcore which works great but is closed source and expensive), as I need to use this data to stabilize a drone. Should i try an older version of isaac ros? Or are there alternatives?I already tried RTAB-MAP on ros1 melodic, but the odom data was too slow (3-7 hz), and ORBSLAM3 (too nooby to figure out how to publish Nav_msgs/Odometry message).
