cmake_minimum_required(VERSION 2.8.3)
project(feature_extraction)

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
find_package(catkin REQUIRED COMPONENTS
  anchor_msgs
  anchor_utils
  cv_bridge
  image_transport
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Find OpenCV 
if(OpenCV_VERSION_MAJOR GREATER 2)
  find_package(OpenCV 3 REQUIRED)
else()
  find_package(OpenCV 2 REQUIRED)
endif()
message( STATUS "Using OpenCV version: " ${OpenCV_VERSION})


#MESSAGE( STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS} )
#MESSAGE( STATUS "OpenCV_LIBRARIES: " ${OpenCV_LIBRARIES} )
#MESSAGE( STATUS "OpenCV_LIBRARY_DIRS: " ${OpenCV_LIBRARY_DIRS} )
#MESSAGE( STATUS "OpenCV_VERSION: " ${OpenCV_VERSION} )

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
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS}
#  LIBRARIES feature_extraction
  CATKIN_DEPENDS anchor_msgs anchor_utils cv_bridge image_transport roscpp
  DEPENDS OpenCV
)

###########
## Build ##
###########
#set(catkin_INCLUDE_DIRS "/opt/ros/kinetic/include/" "/home/aspo/Workspace/devel/include/")
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(extraction src/feature_extraction.cpp src/visual_features.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(extraction
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

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
# install(TARGETS feature_extraction feature_extraction_node
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

