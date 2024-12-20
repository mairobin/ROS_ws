cmake_minimum_required(VERSION 3.0.2)
project(serve_humanity)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_commander
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  roscpp
  ur_kinematics
)

catkin_package(CATKIN_DEPENDS CATKIN_DEPENDS ur_gazebo ur_kinematics roscpp std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serve_humanity
#  CATKIN_DEPENDS rospy std_msgs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ur5_kinematics_node src/ur5_kinematics_node.cpp)

target_link_libraries(ur5_kinematics_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/sine_wave_joint_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
