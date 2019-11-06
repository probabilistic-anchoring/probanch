cmake_minimum_required(VERSION 2.8.3)
project(object_segmentation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS
  anchor_msgs
  console
  cv_bridge
  geometry_msgs
  hand_tracking
  image_geometry
  image_transport
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  rostime
  sensor_msgs
  std_msgs 
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
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
  INCLUDE_DIRS include
#  LIBRARIES segmentation
  CATKIN_DEPENDS  anchor_msgs console roscpp rospy hand_tracking rostime std_msgs geometry_msgs sensor_msgs message_filters cv_bridge image_geometry image_transport pcl_conversions pcl_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(object_segmentation src/object_segmentation.cpp src/pcl_segmentation.cpp)
add_executable(object_calibration src/object_calibration.cpp src/pcl_segmentation.cpp)

add_dependencies(object_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(object_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries( object_segmentation 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries( object_calibration
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

# Communcation test
#add_executable(communcation_node src/communcation_node.cpp)
#target_link_libraries( communcation_node
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}  
#  ${PCL_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  )
#add_library(communcation_nodelet SHARED src/communcation_nodelet.cpp)
#target_link_libraries( communcation_nodelet
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}  
#  ${PCL_LIBRARIES}
#${OpenCV_LIBRARIES}
#  )
#if(catkin_EXPORTED_LIBRARIES)
#  add_dependencies(communcation_nodelet ${catkin_EXPORTED_LIBRARIES})
#endif()

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
# install(TARGETS object_segmentation object_segmentation_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_segmentation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
