# The Anchoring System #

This repository contains the source code of the anchoring system. The overall system is (or it will eventually be) divided into the following individual ROS packages:

* **anchoring**: the main anchoring management system.
* **anchor_caffe**: an ROS wrapper for the Caffe framework (for object recognition/classification).
* **anchor_reasoning**: a package that handles the logical reasoning
* **anchor_msgs**: a separate package for all anchor specific ROS messages.
* **anchor_utils**: a package of libraries and tools, e.g. a separate wrapper library for accessing a MongoDB database.
* **display**: a package used for displaying of the results, both resulting anchors and intermediate results throughout the perceptual pipeline.
* **perceptual_pipeline**: a package for all processing (object segmentation, feature extraction, etc.) of sensor data.

## Dependencies ##

The code has been written and tested in Ubuntu 18.04 together with ROS Melodic. However, the code does not have any OS- or ROS-specific dependencies (except for standard libraries), and should, therefore, work fine even on other Ubuntu distributions together with other ROS versions. Have a look a the more [detailed installation instructions](INSTALL.md)


## Tutorial ##
Coming soon.
