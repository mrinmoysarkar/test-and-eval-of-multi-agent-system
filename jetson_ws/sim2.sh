cd $PX4_FIRMWARE_HOME
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world
