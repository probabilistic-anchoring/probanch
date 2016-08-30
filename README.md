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

Download the repository to your `catkin` workspace and compile, e.g.:

```
$ cd <path/to/workspace>/src
$ git clone https://<user>@bitbucket.org/reground/anchoring.git
$ cd ..
$ catkin_make
```

Note, to prevent `catkin` from compiling one (or more) package(s), simply add an empty `CATKIN_IGNORE` to the package directory (`CATKIN_IGNORE` is added to `.gitignore`, so all `CATKIN_IGNORE` files will only exist locally). For example, to prevent `catkin` from compiling the perceptual_piple package:

```
$ cd <path/to/workspace>/src/anchoring/perceptual_pipeline
$ touch CATKIN_IGNORE
```
