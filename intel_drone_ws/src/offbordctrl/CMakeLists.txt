cmake_minimum_required(VERSION 3.1.0)
project(offbordctrl)
#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-I/usr/include/python2.7)
#add_compile_options(-lpython2.7)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
 # pcl_conversions
 # pcl_msgs
#  nav_msgs
  #librealsense
#  PCL 1.2 REQUIRED
)
find_package(octomap REQUIRED)

#link_directories(
#    ${PCL_LIBRARY_DIRS}
#)
#add_definitions(
#    ${PCL_DEFINITIONS}
#)


## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#    pcl_ros_filters
#    pcl_ros_io
#    pcl_ros_tf
  CATKIN_DEPENDS
  roscpp 
  rospy 
  std_msgs
  #pcl_conversions
  #pcl_msgs
    
#  nav_msgs
#  DEPENDS system_lib
  #librealsense
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}

)
include_directories(${OCTOMAP_INCLUDE_DIRS})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/beginner_tutorials.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_node src/beginner_tutorials_node.cpp)

#add_executable(offbord_node src/offb_node.cpp)
add_executable(setpoint_pub src/pub_setpoints.cpp)
add_executable(pub_trajectory src/pub_trajectory.cpp)
add_executable(hover src/hover.cpp)
add_executable(param_update src/param_update.cpp)
add_executable(flight_path src/flight_path.cpp)
add_executable(Test src/Test.cpp)
add_executable(test2 src/test2.cpp)
add_executable(test3 src/test3.cpp)
add_executable(dataLogger src/dataLogger.cpp)
add_executable(dataLogger_vicon src/dataLogger_vicon.cpp)
add_executable(executeScenario src/executeScenario.cpp)
add_executable(circular_trajectory src/circular_trajectory.cpp)
add_executable(set_home src/setHome.cpp)
add_executable(position_controller src/position_controller.cpp)
add_executable(quad_tf src/quad_tf.cpp)
add_executable(hover_and_fly src/hover_and_fly.cpp)
#add_executable(cameraTest src/cameraTest.cpp)
#add_executable(depth2pointcloud src/rs-depth-to-pointcloud.cpp)
add_executable(octree_simple_example src/octree_simple_example.cpp)
#add_executable(rs_camera2octmap src/rs_camera2octmap.cpp)
#add_executable(rs_camera2octmapV2 src/rs_camera2octmapV2.cpp)
add_executable(friday_demo src/friday_demo.cpp)
add_executable(octomap-test src/octomap-test.cpp)

#target_link_libraries(offbord_node ${catkin_LIBRARIES} -lpython2.7)
target_link_libraries(setpoint_pub ${catkin_LIBRARIES})
target_link_libraries(pub_trajectory ${catkin_LIBRARIES})
target_link_libraries(hover ${catkin_LIBRARIES})
target_link_libraries(param_update ${catkin_LIBRARIES})
target_link_libraries(flight_path ${catkin_LIBRARIES})
target_link_libraries(Test ${catkin_LIBRARIES})
target_link_libraries(test2 ${catkin_LIBRARIES})
target_link_libraries(test3 ${catkin_LIBRARIES})
target_link_libraries(dataLogger ${catkin_LIBRARIES})
target_link_libraries(dataLogger_vicon ${catkin_LIBRARIES})
target_link_libraries(executeScenario ${catkin_LIBRARIES})
target_link_libraries(circular_trajectory ${catkin_LIBRARIES})
target_link_libraries(quad_tf ${catkin_LIBRARIES})
#target_link_libraries(cameraTest ${PCL_LIBRARIES})
target_link_libraries(friday_demo ${catkin_LIBRARIES})

target_link_libraries(set_home ${catkin_LIBRARIES})
target_link_libraries(position_controller ${catkin_LIBRARIES})
target_link_libraries(hover_and_fly ${catkin_LIBRARIES})
#target_link_libraries(depth2pointcloud ${catkin_LIBRARIES} -lrealsense2 -lglfw -lGL -lGLU)
target_link_libraries(octree_simple_example ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_link_libraries(octomap-test ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
#target_link_libraries(rs_camera2octmap ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} -lrealsense2 -lglfw -lGL -lGLU)
#target_link_libraries(rs_camera2octmapV2 ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} -lrealsense2 -lglfw -lGL -lGLU)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )




#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
