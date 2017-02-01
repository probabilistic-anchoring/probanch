cmake_minimum_required(VERSION 2.8.3)
project(anchor_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Anchor.msg
   AnchorArray.msg
   CaffeAttribute.msg
   ClusterArray.msg
   ColorAttribute.msg  
   Contour.msg  
   DescriptorAttribute.msg  
   Display.msg
   DisplayArray.msg
   Inference.msg      # Logic
   InferenceArray.msg # Logic
   MovementArray.msg  
   Object.msg 
   ObjectArray.msg 
   Point2d.msg
   PositionAttribute.msg  
   ShapeAttribute.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   AnchorRequest.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES anchor_msgs
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)