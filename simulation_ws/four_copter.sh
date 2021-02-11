cd $(pwd)/src/Firmware/

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(pwd)/Tools/sitl_gazebo/build
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/Tools/sitl_gazebo/models
export SITL_GAZEBO_PATH=$(pwd)/Tools/sitl_gazebo
# roslaunch px4 four_uav_mavros_sitl_custom.launch
# roslaunch pie multi_copter_sim.launch
# roslaunch px4 multi_uav_mavros_sitl_sdf.launch
roslaunch pie four_uav_mavros_sitl_custom.launch




