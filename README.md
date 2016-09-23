# The Anchoring System #

This repository contains the source code of the anchoring system. The overall system is (or it will eventually be) divided into the following individual ROS packages:

* **anchoring**: the main anchoring management system.
* **anchor_caffe**: an ROS wrapper for the Caffe framework (for object recognition/classification).
* **anchor_msgs**: a separate package for all anchor specific ROS messages.
* **perceptual_pipeline**: a package for all processing (object segmentation, feature extraction, etc.) of sensor data.

## Dependencies ##

The code has been written and tested in Ubuntu 14.04 together with ROS Indigo. However, the code does not have any OS- or ROS-specific dependencies (except for standard libraries), and should, therefore, work fine even on later Ubuntu distributions together with later ROS versions.

*Also, see each individual package for specific dependencies.*

## Install ##

First of all, the ROS environment is required for the overall system to integrate and communicate. In Ubuntu 10.04, install (and create a `catkin` workspace) by:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop
$ sudo rosdep init
$ rosdep update
$ source /opt/ros/indigo/setup.bash
$ mkdir -p ~/<workspace>/src
$ cd ~/<workspace>/src
$ catkin_init_workspace
```

Download the repository to your `catkin` workspace and compile, e.g.:

```
$ cd <path/to/workspace>/src
$ git clone --recursive https://<user>@bitbucket.org/reground/anchoring.git
$ cd ..
$ catkin_make
```

Note, to prevent `catkin` from compiling one (or more) package(s), simply add an empty `CATKIN_IGNORE` to the package directory (`CATKIN_IGNORE` is added to `.gitignore`, so all `CATKIN_IGNORE` files will only exist locally). For example, to prevent `catkin` from compiling the **perceptual_pipeline** package:

```
$ cd <path/to/workspace>/src/anchoring/perceptual_pipeline
$ touch CATKIN_IGNORE
```