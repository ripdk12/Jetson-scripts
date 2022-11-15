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

### 2. librealsense - optional

Following the guide on https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam#docker, download the git repos required for your application.
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
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
```
```
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
```
```
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
```
```
git clone https://github.com/IntelRealSense/realsense-ros.git
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

#### Enabling microRTPS client on Cube Orange+

https://github.com/PX4/PX4-Autopilot/issues/20187

https://github.com/PX4/PX4-Autopilot/pull/20304

#### gg
```
  876  git clone --branch pr-cubeorange+ --recursive https://github.com/CubePilot/PX4-Autopilot.git
  
  877  cd PX4-Autopilot/
  
  878  make cubepilot_cubeorangeplus
  
  879  make cubepilot_cubeorangeplus_default upload
  
  880  pip3 install kconfiglib
  
  881  make cubepilot_cubeorangeplus_default upload
  
  882  pip3 install --user jsonschema
  
  883  make cubepilot_cubeorangeplus_default upload
  
  884  git tag --help
 
  885  git tag v1.15.0
  
  886  git tag -a v1.15.0
  
  887  git tag
  
  888  make cubepilot_cubeorangeplus_default upload
```


