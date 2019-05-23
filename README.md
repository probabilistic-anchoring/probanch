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

The code has been written and tested in both Ubuntu 16.04 (Xenial) together with ROS Kinetic, as well as Ubuntu 18.04 (Bionic) together with ROS Melodic. However, the code does not have any OS- or ROS-specific dependencies (except for standard libraries), and should, therefore, work fine even on other Ubuntu releases together with other ROS distributions. Have a look a the more [detailed installation instructions](INSTALL.md)


### Required

#### 1. Robot Operating System (ROS)

#### 2. OpenCV (Open Source Computer Vision Library)

#### 3. MongoDB Database

In order to persistently store and maintain anchored objects is a [MongoDB NoSQL database server](https://www.mongodb.com/) integrated and used. More specifically, this the *utils* directory contains a *database* package which is compiled as a library which, currently, contains a generic database interface for accessing a [MongoDB NoSQL database](https://www.mongodb.com/) for persistent storage of anchors, log data, etc.
This library has been rewriten for the latest C++11 drivers for [MongoDB](http://mongodb.github.io/mongo-cxx-driver/mongocxx-v3/). Latest version of MongoDB Community Edition database, together with all C and C++ drivers, can be installed through the following steps:

* Install the database:
```bash
$ sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 0C49F3730359A14518585931BC711F9BA15703C6
$ echo "deb [ arch=amd64 ] http://repo.mongodb.org/apt/ubuntu trusty/mongodb-org/3.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-3.4.list
$ sudo apt-get update
$ sudo apt-get install -y mongodb-org
```

* Install MongoDB C drivers:
```
$ sudo apt-get install pkg-config libssl-dev libsasl2-dev git gcc automake autoconf libtool
$ git clone https://github.com/mongodb/mongo-c-driver.git
$ cd mongo-c-driver
$ git checkout 1.6.0  # To build latest stable release
$ ./autogen.sh --with-libbson=bundled --enable-static
$ make
$ sudo make install
```

* Install MongoDB C++ drivers:
```
$ git clone https://github.com/mongodb/mongo-cxx-driver.git --branch releases/stable --depth 1
$ cd mongo-cxx-driver/build
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
$ sudo make EP_mnmlstc_core # For MNMLSTC polyfill
$ make && sudo make install
```
*Note, make sure that library path of the install libraries, e.g.* `/usr/local/lib`*, is part of your* `LD_LIBRARY_PAHT`.

### Optional

#### 1. Nvidia CUDA

For optimal performance, this framework is using Nvidia CUDA libraries for both handling the processing of input sensory RGB-D data, as well as semantically classifying perceived objects. In the latter case is further the Caffe deep learning framework utilized (likewise for optimized system performance).



## Tutorial ##
Coming soon.
