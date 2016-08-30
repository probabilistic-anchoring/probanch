# The Anchoring System #

This repository contains the source code of the anchoring system. The overall system is (or it will eventually be) divided into the following individual ROS packages:

* [anchoring](https://bitbucket.org/reground/anchoring/src/e7ff087a0d769887a99c9fa3e521b6d1d1127ec1/anchoring/?at=master): the main anchoring management system.
* anchor_caffe: an ROS wrapper for the Caffe framework (for object recognition/classification).
* [anchor_msgs](https://bitbucket.org/reground/anchoring/src/e7ff087a0d769887a99c9fa3e521b6d1d1127ec1/anchor_msgs/?at=master): a separate package for all anchor specific ROS messages.
* perceptual_pipeline: a package for all processing (object segmentation, feature extraction, etc.) of sensor data.

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