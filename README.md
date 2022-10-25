# Jetson-scripts
Convenience scripts and Dockerfiles for Jetson Nano

## Useful Links

#### Guide for flashing and booting from ssd/usb device
https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3261/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html#wwpID0E0NK0HA

#### Solution to librealsense2 certificate issue
https://github.com/IntelRealSense/librealsense/issues/10980

#### error msg

--- stderr: realsense2_camera          
CMake Error at /workspaces/isaac_ros-dev/install/realsense2_camera_msgs/share/realsense2_camera_msgs/cmake/ament_cmake_export_targets-extras.cmake:18 (message):
  Failed to find exported target names in
  '/workspaces/isaac_ros-dev/install/realsense2_camera_msgs/share/realsense2_camera_msgs/cmake/export_realsense2_camera_msgs__rosidl_generator_cExport.cmake'
Call Stack (most recent call first):
  /workspaces/isaac_ros-dev/install/realsense2_camera_msgs/share/realsense2_camera_msgs/cmake/realsense2_camera_msgsConfig.cmake:41 (include)
  CMakeLists.txt:96 (find_package)


---
Failed   <<< realsense2_camera [7.39s, exited with code 1]
