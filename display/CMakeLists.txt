cmake_minimum_required(VERSION 2.8.3)
project(display)

# Check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  anchor_msgs
  cv_bridge
  dc_msgs
  image_geometry
  image_transport
  roscpp
  sensor_msgs
)

# Prevent Cude library from giving a linking error
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

# Find OpenCV 
if(OpenCV_VERSION_MAJOR GREATER 2)
  find_package(OpenCV 3 REQUIRED)
else()
  find_package(OpenCV 2 REQUIRED)
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

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
#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES display
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  /usr/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(display
#   src/${PROJECT_NAME}/display.cpp
# )

## Add cmake target dependencies of the library
# add_dependencies(display ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(anchor_viewer src/anchor_viewer.cpp)
add_executable(img2str src/img_to_base64.cpp)
add_executable(anchor_recorder src/anchor_recorder.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(display_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(anchor_viewer 
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
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
