cmake_minimum_required(VERSION 2.8.3)
project(features)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ml
  pcl_conversions
  pcl_ros
  roscpp
  roslib
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

# Find the Point Clude Library (PCL)
find_package(PCL REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS features2d)
message( STATUS "Using OpenCV version: " ${OpenCV_VERSION})

# Find OpenCV
#find_package(OpenCV REQUIRED COMPONENTS core highgui)
#message( STATUS "Using OpenCV version: " ${OpenCV_VERSION})

## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

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
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES spatial visual 
  CATKIN_DEPENDS geometry_msgs roscpp roslib
  DEPENDS OpenCV PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ libraries
add_library( visual src/color_features.cpp src/keypoint_features.cpp)
add_library( spatial src/spatial_features.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( visual
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries( spatial
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# For testing
#add_executable( feature_test src/db_test.cpp)
#target_link_libraries( feature_test
#  ${PROJECT_NAME} 
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
