#!/bin/bash

## Bash script for setting up a PX4 development environment on Ubuntu LTS (16.04).
## It can be used for installing simulators (only) or for installing the preconditions for Snapdragon Flight or Raspberry Pi.
##
## Installs:
## - Common dependencies and tools for all targets (including: Ninja build system, pyulog)
## - jMAVSim simulator dependencies
## - AUTERION/VIO ros package and dependencies

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

# Ubuntu Config
echo "Remove modemmanager"
sudo apt-get remove modemmanager -y
echo "Add user to dialout group for serial port access (reboot required)"
sudo usermod -a -G dialout $USER

# Common dependencies
echo "Installing common dependencies"
sudo apt-get update -y
sudo apt-get install git zip cmake build-essential genromfs ninja-build exiftool astyle -y
# make sure xxd is installed, dedicated xxd package since Ubuntu 18.04 but was squashed into vim-common before
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y
# Required python packages
sudo apt-get install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial pyyaml
# optional python tools
sudo -H pip3 install pyulog

# jMAVSim simulator dependencies
echo "Installing jMAVSim simulator dependencies"
sudo apt-get install ant openjdk-11-jdk openjdk-11-jre -y




# ROS Melodic
## Gazebo simulator dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y


sudo apt-get update

rosdep update
## Setup environment variables
rossource="source /opt/ros/melodic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource

## Install rosinstall and other dependencies
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y

# MAVROS: https://dev.px4.io/en/ros/mavros_installation.html
## Install dependencies
sudo apt-get install -y python-catkin-tools python-rosinstall-generator ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-pcl-ros ros-melodic-imu-filter-madgwick ros-melodic-rtabmap ros-melodic-rtabmap-ros ros-melodic-robot-localization

cd ~/catkin_ws
catkin init
wstool init src

## Install MAVLink
###we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

## Build MAVROS
### Get source (upstream - released)
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

### Setup workspace & install deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
if ! rosdep install --from-paths src --ignore-src -y; then
    # (Use echo to trim leading/trailing whitespaces from the unsupported OS name
    unsupported_os=$(echo $(rosdep db 2>&1| grep Unsupported | awk -F: '{print $2}'))
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y --os ubuntu:bionic
fi

if [[ ! -z $unsupported_os ]]; then
    >&2 echo -e "\033[31mYour OS ($unsupported_os) is unsupported. Assumed an Ubuntu 18.04 installation,"
    >&2 echo -e "and continued with the installation, but if things are not working as"
    >&2 echo -e "expected you have been warned."
fi

#Install geographiclib
sudo apt install geographiclib-tools -y
echo "Downloading dependent script 'install_geographiclib_datasets.sh'"
# Source the install_geographiclib_datasets.sh script directly from github
install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'install_geographiclib_datasets.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
sudo bash -c "$install_geo"

## Build!
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc;
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source




##MERGING WITH Realsenseinstallscript####################################
cd ~/catkin_ws/src



## Bash script for setting up librealsense2, RealSense ROS-wrapper and dependencies
## Forked from JetsonHacksNano
## Installs:
## - librealsense2 2.50
## - realsense-ros 2.3.2

echo "Register the server key for librealsense.."

sudo apt-key adv --keyserver keyserver.ubuntu.com  --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

#sudo apt install apt-utils -y

sudo apt install librealsense2=2.50.0-0~realsense0.6127 librealsense2-gl=2.50.0-0~realsense0.6127 librealsense2-net=2.50.0-0~realsense0.6127 librealsense2-utils=2.50.0-0~realsense0.6127 librealsense2-dev=2.50.0-0~realsense0.6127 librealsense2-udev-rules=2.50.0-0~realsense0.6127 -y




