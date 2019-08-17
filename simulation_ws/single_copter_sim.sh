cd $PX4_FIRMWARE_HOME
#source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
#source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
#roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/solo_custom.world

#roslaunch px4 multi_uav_mavros_sitl.launch

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
roslaunch px4 single_uav_mavros_sitl_custom.launch
