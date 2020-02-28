# **ProbAnch**: a Probabilistic Perceptual Anchoring Framework #

This repository contains the source code of the **anchoring framework** used to *" ...create and maintain the correspondence between symbols and sensor data that refer to the same physical objects"*, as described in [[1]](#markdown-header-references). The overall framework is a modularized framework consisting of several individual programs which are utilizing the infrastructure and communication protocols found in the [ROS (Robot Operating System)](http://wiki.ros.org/) environment.




<!-- <a href="{https://player.vimeo.com/video/388874421}" title="Transitive Occlusion"><img src="{image-url}" alt="Transitive Occlusion" /></a> -->

[![ProbAnch - Transitive Occlusion](http://i.imgur.com/7YTMFQp.png)](https://vimeo.com/388874421 "ProbAnch - Transitive Occlusion")


<!-- <iframe src="https://player.vimeo.com/video/388874421" width="640" height="564" frameborder="0" allow="autoplay; fullscreen" allowfullscreen></iframe> -->

<iframe  title="YouTube video player" width="480" height="390" src="http://www.youtube.com/watch?v=TheVideoID?autoplay=1" frameborder="0" allowfullscreen></iframe>


## References ##

__[1]__ A. Persson, P. Zuidberg Dos Martires, L. De Raedt and A. Loutfi: [Semantic Relational Object Tracking](https://arxiv.org/abs/1902.09937). In: IEEE Transactions on Cognitive and Developmental Systems, 2019.
__[2]__ P. Zuidberg Dos Martires, A. Persson, N. Kumar, A. Loutfi and L. De Raedt  [Symbolic Learning and Reasoning with Noisy Data for Probabilistic Anchoring](https://arxiv.org/abs/2002.10373)




## Framework Architecture ##

The overall system is divided into the following subfolders, which each contains one or many individual ROS packages:

* [**anchoring**](https://bitbucket.org/reground/anchoring/src/master/anchoring/): the main anchoring management system.
* [**display**](https://bitbucket.org/reground/anchoring/src/master/display/): a package used for displaying of the results, both resulting anchors and intermediate results throughout the perceptual pipeline.
* [**grounding**](https://bitbucket.org/reground/anchoring/src/master/grounding/): contains (currently) only a ROS wrapper for the Caffe framework for object classification.
* [**messages**](https://bitbucket.org/reground/anchoring/src/master/messages/): a folder for separate package(s) for all anchor specific ROS messages.
* [**perception**](https://bitbucket.org/reground/anchoring/src/master/perception/): a seperate folder for all package for handling and processing of sensor data (e.g., object segmentation, feature extraction, etc.).
* [**reasoning**](https://bitbucket.org/reground/anchoring/src/master/reasoning/): a package that handles the logical reasoning.
* [**utils**](https://bitbucket.org/reground/anchoring/src/master/utils/): a package of libraries and tools, e.g. a separate wrapper library for accessing a MongoDB database.
<!-- * [**bagfiles**](https://bitbucket.org/reground/anchoring/src/master/bagfiles/): contains a couple of example bagfiles (should, however, be kept as a minimal folder in order to prevent the repository to exceed the storage capacity). -->
_____________________


## Dependencies ##

The code has been written and tested in both Ubuntu 16.04 (Xenial) together with ROS Kinetic, as well as Ubuntu 18.04 (Bionic) together with ROS Melodic. However, the code does not have any OS- or ROS-specific dependencies (except for standard libraries), and should, therefore, work fine even on other Ubuntu releases together with other ROS distributions.

* [Basic Requirements](#markdown-header-basic-requirements)
* [Optimal Performance](#markdown-header-optimal-performance)

For more details, have a look a the more [detailed installation instructions](INSTALL.md)


### __Basic Requirements:__

The following dependencies are required for a basic installation of the framework:

#### 1. MongoDB Database Server

For this framework, a document-oriented NoSQL [MongoDB Database server](https://www.mongodb.com/) is integrated and used in order to facilitate persistent storage and maintenance of anchored objects.
More specifically, the `utils` directory contains a `database` package that is compiled as a library which, currently, contains a *generic database interface* for seamless access and communication with the database in order to store and retrieve anchored objects, log data, etc. This library has been rewritten for the latest C++11 drivers for [MongoDB](http://mongodb.github.io/mongo-cxx-driver/mongocxx-v3/).  The latest version of the [MongoDB Community Edition database](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/), together with all C and C++ drivers, can be installed through the following steps:

* __Install the database:__

        #!sh
        sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 0C49F3730359A14518585931BC711F9BA15703C6
        echo "deb [ arch=amd64 ] http://repo.mongodb.org/apt/ubuntu trusty/mongodb-org/3.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-3.4.list
        sudo apt-get update
        sudo apt-get install -y mongodb-org

* __Install MongoDB C drivers:__

        #!sh
        sudo apt-get install pkg-config libssl-dev libsasl2-dev git gcc automake autoconf libtool cmake
        git clone https://github.com/mongodb/mongo-c-driver.git
        cd mongo-c-driver
        mkdir cmake-build && cd cmake-build
        cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF ..
        make
        sudo make install

* __Install MongoDB C++ drivers:__

        #!sh
        git clone https://github.com/mongodb/mongo-cxx-driver.git --branch releases/stable --depth 1
        cd mongo-cxx-driver/build
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
        sudo make EP_mnmlstc_core # For MNMLSTC polyfill
        make && sudo make install

#### 2. OpenCV (Open Source Computer Vision Library)

The [OpenCV (Open Source Computer Vision library)](https://opencv.org/) is the primary library used by this framework for the processing of visual sensory data. The framework has been installed and tested together with [OpenCV 3.4](https://github.com/opencv/opencv). To install the latest stable release of OpenCV 3.4, proceed with the following steps:

* __Required system dependencies:__

        #!sh
        sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

* __Optional system dependencies:__

        #!sh
        sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

* __Download and prepare the installation:__

        #!sh
        ~/Download/
        git clone https://github.com/opencv/opencv.git
        git clone https://github.com/opencv/opencv_contrib.git
        cd ~/Download/opencv_contrib && git checkout 3.4.7
        cd ~/Download/opencv && git checkout 3.4.7

* __Compile and install:__

        #!sh
        ~/Download/opencv
        mkdir build && cd build
        cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ ..
        make -j8
        sudo make install

*__Note:__* *make sure that library path of the install libraries, e.g.* `/usr/local/lib`*, is part of your* `LD_LIBRARY_PAHT`.

#### 3. Robot Operating System (ROS)

The appropriate ROS distribution for currently supported Ubuntu LTS releases can be installed through the following steps:

* __Setup software packages and keys:__

        #!sh
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

* __Ubuntu 16.04 (Xenial) installation:__

        #!sh
        sudo apt-get update && sudo apt-get install ros-kinetic-ros-base

* __Ubuntu 18.04 (Bionic) installation:__

        #!sh
        sudo apt-get update && sudo apt-get install ros-melodic-ros-base

* __Initialize rosdep (same for all Ubuntu releases):__

        #!sh
        sudo rosdep init
        rosdep update

This framework is also heavily dependent of the `cv_bridge` ROS package for converting and transporting OpenCV images between submodules (or ROS nodes). However, the `cv_bridge`, installed as part of the larger `vision_opencv` package through the default package manager (e.g., `sudo apt-get install ros-melodic-vision-opencv`), is __only__ built for __Python 2.7__.

*__Note:__* *installing the* `vision_opencv` *package through the use of the package manager, will also install a minimal "default" version of OpenCV.*

To instead build `cv_bridge` (as part of larger the `vision_opencv` package) using the custom installation of OpenCV (installed in *Step 2*), together with either __Python 2.7__ or __Python 3.x__:

* __Common preparation step:__

        #!sh
        mkdir -p ~/Workspaces/catkin_ws && cd ~/Workspaces/catkin_ws

* __Python 3.x (in this case, Python 3.6) dependencies and configuration (NOT required for Python 2.7):__

        #!sh
        sudo apt-get install python-catkin-tools python3-pip python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml
        sudo -H pip3 install --upgrade pip
        sudo -H pip3 install rospkg catkin_pkg
        mkdir -p ~/Workspaces/catkin_ws && cd ~/Workspaces/catkin_ws
        catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

* __Common installation steps:__

        #!sh
        catkin config --install
        mkdir src && cd src
        git clone -b melodic https://github.com/ros-perception/vision_opencv.git
        cd ~/Workspaces/catkin_ws
        catkin build cv_bridge
        source install/setup.bash --extend

*__Note:__* *the ROS environment variables must be sourced (e.g.,* `source /opt/ros/melodic/setup.bash` *) for the package to build correctly.*

_____________________

### __Optimal Performance:__

#### 1. [Nvidia CUDA](https://developer.nvidia.com/cuda-zone)

For optimal performance, this framework is using Nvidia CUDA libraries for both handling the processing of input sensory RGB-D data, as well as semantically classifying perceived objects. In the latter case is further the Caffe deep learning framework utilized (likewise for optimized system performance).

#### 2. [Caffe](https://caffe.berkeleyvision.org/) Deep Learning Framework

For optimal performance, this framework is using Nvidia CUDA libraries for both handling the processing of input sensory RGB-D data, as well as semantically classifying perceived objects. In the latter case is further the Caffe deep learning framework utilized (likewise for optimized system performance).

_____________________

## Installation ##

Your are now ready to clone and install the anchoring framework.

    #!sh
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    git clone --recursive https://<user>@bitbucket.org/reground/anchoring.git
_____________________


## Tutorial ##
Coming soon.
_____________________
