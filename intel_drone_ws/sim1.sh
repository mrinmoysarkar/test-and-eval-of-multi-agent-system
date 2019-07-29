#roslaunch px4 multi_uav_mavros_sitl.launch

cd $PX4_FIRMWARE_HOME
no_sim=1 make posix_sitl_default gazebo

