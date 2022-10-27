# Jetson-scripts
Convenience scripts and Dockerfiles for Jetson Nano

## Step-by-step
### 1. Flash Jetson
On your Jetson host machine, running Ubuntu 18.04/20.04 - depending on desired Jetpack version (4.x/5.x)
Download the Nvidia SDK Manager: https://developer.nvidia.com/drive/sdk-manager
Open the program and select the relevant board and flash
should the flash fail (as mine did), refer to https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3261/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html#wwpID0E0PI0HA.
The following worked for me (flashing to nvme drive):
```
systemctl stop udisks2.service
sudo ./tools/kernel_flash/l4t_initrd_flash.sh jetson-xavier-nx-devkit-emmc nvme0n1p1
systemctl start udisks2.service
```
When finished remember to also install CUDA and NVIDIA Container Runtime SDK components in order to run docker containers with Nvidia runtime support.

### 2. librealsense
Following the guide on https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam#docker, download the git repos required for your application
NOTE: librealsense2 might be required to be installed outside of the container in order to get the RealSense camera to work. (Unsure about this)
``` 
chmod +x installlibrealsense.sh
chmod +x realsense_dependencies.sh
sudo ./realsense_dependencies.sh
sudo ./installlibrealsense.sh
```
This will take about 45 minutes depending on your system.

### 3. Isaac ROS Visual SLAM
Now we should be ready to run the docker images from the isaac ros github repos, but first, we need to setup the development environment as explained here: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md
Next follow this guide to setup the realsense camera: https://github.com/NVIDIA-ISAAC-ROS/.github/blob/main/profile/realsense-setup.md
Lastly follow this quickstart setup for Isaac ROS Visual SLAM: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam#quickstart


```
 cd /tmp && \
 git clone https://github.com/IntelRealSense/librealsense && \
 cd librealsense && \
 ./scripts/setup_udev_rules.sh
```

```
mkdir -p ~/workspaces/isaac_ros-dev/src && cd ~/workspaces/isaac_ros-dev/src
```
```
git clone -b release-dp-1.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
```
```
git clone -b release-dp-1.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
```
```
git clone -b release-dp-1.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
```
```
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta
```

```
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts && \
touch .isaac_ros_common-config && \
echo CONFIG_IMAGE_KEY=humble.nav2.realsense > .isaac_ros_common-config
```
## Useful Links

#### Guide for flashing and booting from ssd/usb device
https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3261/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html#wwpID0E0NK0HA

#### Solution to librealsense2 certificate issue
https://github.com/IntelRealSense/librealsense/issues/10980

#### gg

I have setup the docker development environment with isaac_ros_visual_slam and RealSense D435i camera on a Xavier NX (JP5.0.2), following the guide on github, however, the results are extremely poor as shown in the picture. 
I believe it's similar to the issue described here: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/issues/54


I am mostly interested in a free solution for visual inertial odometry, (currently using slamcore which works great but is closed source and expensive), as I need to use this data to stabilize a drone. Should i try an older version of isaac ros? Or are there alternatives?I already tried RTAB-MAP on ros1 melodic, but the odom data was too slow (3-7 hz), and ORBSLAM3 (too nooby to figure out how to publish Nav_msgs/Odometry message).
