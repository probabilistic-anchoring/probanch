cmake_minimum_required(VERSION 2.8.3)
project(database)

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
  roscpp
  roslib
)

## System dependencies are found with CMake's conventions
find_package(libbsoncxx REQUIRED)
find_package(libmongocxx REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core highgui ml)
message( STATUS "Using OpenCV version: " ${OpenCV_VERSION})

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
  INCLUDE_DIRS include ${LIBMONGOCXX_INCLUDE_DIRS} ${LIBBSONCXX_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBMONGOCXX_INCLUDE_DIRS}
  ${LIBBSONCXX_INCLUDE_DIRS}
)

# Link the external libraries
link_directories(
  ${LIBMONGOCXX_LIBRARY_DIRS}
  ${LIBBSONCXX_LIBRARY_DIRS}
)

## Declare a C++ library
add_library( ${PROJECT_NAME} src/database.cpp )

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME}
  ${LIBMONGOCXX_LIBRARIES}
  ${LIBBSONCXX_LIBRARIES}
)

# Simple database cleaner
add_executable(cleaner src/db_clean.cpp)
target_link_libraries( cleaner
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Testing
add_executable(db_test src/db_test.cpp)
target_link_libraries( db_test
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

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
