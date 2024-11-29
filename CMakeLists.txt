cmake_minimum_required(VERSION 3.0.2)
project(tello_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Declare ROS messages, services and actions ##
add_message_files(
  FILES
  system_vars_msg.msg
)

## Generate messages
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

## Declare ROS dynamic reconfigure parameters ##
generate_dynamic_reconfigure_options(
  cfg/dynamic_gains.cfg
)

## catkin specific configuration ##
catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add executable
# add_executable(${PROJECT_NAME}_node src/keyboard_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#############
## Install ##
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
