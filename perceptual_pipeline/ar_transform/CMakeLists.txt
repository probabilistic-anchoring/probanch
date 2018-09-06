cmake_minimum_required(VERSION 2.8.3)
project(ar_transform)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  geometry_msgs
  image_geometry
  image_transport
  message_generation
  rospy
  rostime
  sensor_msgs
  std_msgs
  tf
  cv_bridge

)



## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#)

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
#generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   sensor_msgs
#   std_msgs
#)








include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
