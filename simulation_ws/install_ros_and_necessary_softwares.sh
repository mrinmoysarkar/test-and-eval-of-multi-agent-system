#!/bin/bash

## Bash script for setting up ROS Melodic (with Gazebo 9) development environment for PX4 on Ubuntu LTS (18.04). 
## It installs the common dependencies for all targets (including Qt Creator)
##
## Installs:
## - Common dependencies libraries and tools as defined in `ubuntu_sim_common_deps.sh`
## - ROS Melodic (including Gazebo9)
## - MAVROS

if [[ $(lsb_release -sc) == *"xenial"* ]]; then
  echo "OS version detected as $(lsb_release -sc) (16.04)."
  echo "ROS Melodic requires at least Ubuntu 18.04."
  echo "Exiting ...."
  return 1;
fi

# echo "Downloading dependent script 'ubuntu_sim_common_deps.sh'"
# # Source the ubuntu_sim_common_deps.sh script directly from github
# common_deps=$(wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh -O -)
# wget_return_code=$?
# # If there was an error downloading the dependent script, we must warn the user and exit at this point.
# if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'ubuntu_sim_common_deps.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# # Otherwise source the downloaded script.
# . <(echo "${common_deps}")

# Ubuntu Config
echo "Remove modemmanager"
sudo apt remove modemmanager -y
echo "Add user to dialout group for serial port access (reboot required)"
sudo usermod -a -G dialout $USER


# Common dependencies
echo "Installing common dependencies"
sudo apt update -y
sudo apt install git zip cmake build-essential genromfs ninja-build exiftool astyle -y
# make sure xxd is installed, dedicated xxd package since Ubuntu 18.04 but was squashed into vim-common before
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y


# ROS Melodic
## Gazebo simulator dependencies
sudo apt install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/melodic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt update
## Get ROS/Gazebo
sudo apt install ros-melodic-desktop-full -y

# sudo apt install python-rosdep2
## Initialize rosdep
# sudo rosdep init
# rosdep update
## Setup environment variables
rossource="source /opt/ros/melodic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource

## Install rosinstall and other dependencies
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool python-catkin-tools build-essential -y


# MAVROS: https://dev.px4.io/en/ros/mavros_installation.html
## Install dependencies
# sudo apt-get install python-catkin-tools python-rosinstall-generator -y

sudo rosdep init
rosdep update

sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y
sudo apt install ros-melodic-octomap -y
sudo apt install ros-melodic-octomap-mapping -y
sudo apt install ros-melodic-moveit -y
sudo apt install xmlstarlet -y

# ## Create catkin workspace
# mkdir -p ~/catkin_ws/src
# cd ~/catkin_ws
# catkin init
# wstool init src


# ## Install MAVLink
# ###we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
# rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

# ## Build MAVROS
# ### Get source (upstream - released)
# rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

# ### Setup workspace & install deps
# wstool merge -t src /tmp/mavros.rosinstall
# wstool update -t src
# if ! rosdep install --from-paths src --ignore-src -y; then
#     # (Use echo to trim leading/trailing whitespaces from the unsupported OS name
#     unsupported_os=$(echo $(rosdep db 2>&1| grep Unsupported | awk -F: '{print $2}'))
#     rosdep install --from-paths src --ignore-src --rosdistro melodic -y --os ubuntu:bionic
# fi

# if [[ ! -z $unsupported_os ]]; then
#     >&2 echo -e "\033[31mYour OS ($unsupported_os) is unsupported. Assumed an Ubuntu 18.04 installation,"
#     >&2 echo -e "and continued with the installation, but if things are not working as"
#     >&2 echo -e "expected you have been warned."
# fi

#Install geographiclib
sudo apt install geographiclib -y
echo "Downloading dependent script 'install_geographiclib_datasets.sh'"
# Source the install_geographiclib_datasets.sh script directly from github
install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'install_geographiclib_datasets.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
sudo bash -c "$install_geo"

# ## Build!
# catkin build
# ## Re-source environment to reflect new packages/build environment
# catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
# if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc; 
# else echo "$catkin_ws_source" >> ~/.bashrc; fi
# eval $catkin_ws_source


# # Go to the firmware directory
# clone_dir=~/src
# cd $clone_dir/Firmware


# Required python packages
sudo apt install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial pyyaml
# optional python tools
sudo -H pip install pyulog

rosdep install --from-paths src --ignore-src --rosdistro melodic -y
catkin_make

if grep -Fxq "source $PWD/devel/setup.bash" ~/.bashrc; then echo ROS simulation_ws setup.bash already in .bashrc; 
else echo "source $PWD/devel/setup.bash" >> ~/.bashrc; fi


if grep -Fxq "export MESH_WORKSPACE_PATH=$PWD/src/Firmware/Tools/sitl_gazebo/models/" ~/.bashrc; then echo MESH_WORKSPACE_PATH already in .bashrc; 
else echo "export MESH_WORKSPACE_PATH=$PWD/src/Firmware/Tools/sitl_gazebo/models/" >> ~/.bashrc; fi

if grep -Fxq "export PX4_FIRMWARE_HOME=$PWD/src/Firmware/" ~/.bashrc; then echo PX4_FIRMWARE_HOME already in .bashrc; 
else echo "export PX4_FIRMWARE_HOME=$PWD/src/Firmware/" >> ~/.bashrc; fi

if grep -Fxq "export CATKIN_WORKSPACE_DIR=$PWD" ~/.bashrc; then echo CATKIN_WORKSPACE_DIR already in .bashrc; 
else echo "export CATKIN_WORKSPACE_DIR=$PWD" >> ~/.bashrc; fi

export MESH_WORKSPACE_PATH=$PWD/src/Firmware/Tools/sitl_gazebo/models/
export PX4_FIRMWARE_HOME=$PWD/src/Firmware/
export CATKIN_WORKSPACE_DIR=$PWD

bashsource="source ~/.bashrc"
source ~/.bashrc
eval $bashsource
