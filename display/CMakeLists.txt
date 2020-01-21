cmake_minimum_required(VERSION 2.8.3)
project(display)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  anchor_msgs
  cv_bridge
  reasoning
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  tf2_geometry_msgs
)

# Find OpenCV
find_package(OpenCV REQUIRED)
message( STATUS "Using OpenCV version: " ${OpenCV_VERSION})

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  LIBRARIES display
  CATKIN_DEPENDS anchor_msgs cv_bridge image_transport roscpp sensor_msgs tf2_geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(anchor_viewer src/anchor_viewer.cpp)
add_executable(img2str src/img_to_base64.cpp)
add_executable(anchor_recorder src/anchor_recorder.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(anchor_viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(img2str ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(anchor_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(anchor_viewer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
target_link_libraries(img2str
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
target_link_libraries(anchor_recorder
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
