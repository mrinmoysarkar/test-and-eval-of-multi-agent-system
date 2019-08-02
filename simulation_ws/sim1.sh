#roslaunch px4 multi_uav_mavros_sitl.launch

cd $PX4_FIRMWARE_HOME
#no_sim=1 make posix_sitl_default gazebo
#no_sim=1 make px4_sitl gazebo
DONT_RUN=1 make px4_sitl_default gazebo

