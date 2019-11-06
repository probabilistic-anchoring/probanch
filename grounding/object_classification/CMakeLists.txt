cmake_minimum_required(VERSION 2.8.3)
project(object_classification)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Set var for Caffe/OpenCV implementation (TRUE/FALSE)
set(CAFFE_SUPPORT TRUE)
if( CAFFE_SUPPORT )
  add_definitions( -DINCLUDE_CAFFE_CLASSIFIER )
  message(STATUS "Caffe framework used for classification." )
  
  ## Set Caffe dirs
  if(DEFINED ENV{CAFFE_ROOT})
    set(Caffe_DIR $ENV{CAFFE_ROOT})
    message(STATUS "Caffe root directory: " ${Caffe_DIR})
  else()
    remove_definitions( -DINCLUDE_CAFFE_CLASSIFIER )
    set(CAFFE_SUPPORT FALSE)
    message(STATUS "No Caffe found, using OpenCV instead.")
  endif()
  set(Caffe_INCLUDE_DIRS ${Caffe_DIR}/include ${Caffe_DIR}/distribute/include)
  set(Caffe_LINK_LIBRARAY ${Caffe_DIR}/build/lib)
else()
  message(STATUS "OpenCV used for classification." )
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  anchor_msgs
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
if( CAFFE_SUPPORT )
  find_package(CUDA QUIET)
  if( NOT CUDA_FOUND )
    
    # Set Caffe var for CPU only use
    add_definitions(-DCPU_ONLY=1)
  endif()
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
   FILES
   CaffeService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES anchor_caffe
  CATKIN_DEPENDS anchor_msgs cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
if( CUDA_FOUND AND CAFFE_SUPPORT )
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${Caffe_INCLUDE_DIRS}
  )
elseif( CAFFE_SUPPORT )
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Caffe_INCLUDE_DIRS}
  )
else()
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )
endif()

# Link the Caffe library
link_directories(${Caffe_LINK_LIBRARAY})

## Declare a C++ library
# add_library(anchor_caffe
#   src/${PROJECT_NAME}/anchor_caffe.cpp
# )

## Add cmake target dependencies of the library
# add_dependencies(anchor_caffe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(object_classification src/object_classification.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(object_classification ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
if( CUDA_FOUND AND CAFFE_SUPPORT )
  target_link_libraries(
    object_classification
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    caffe
    glog
  )
elseif( CAFFE_SUPPORT ) 
  target_link_libraries(
    object_classification
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
    caffe
    glog
  )
else()
  target_link_libraries(
    object_classification
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
  )
endif()


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
# install(TARGETS anchor_caffe anchor_caffe_node
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

