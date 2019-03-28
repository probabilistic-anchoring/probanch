## Install ##

First of all, the ROS environment is required for the overall system to integrate and communicate. Follow the instructions on the [project's website](http://wiki.ros.org/ROS/Installation) to install ROS, initialize dependencies, and setup your environment.


Your are now ready to clone the anchoring framework.
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone --recursive https://<user>@bitbucket.org/reground/anchoring.git
```

Before building the project we need to install all the dependencies for the subpackages listed above.

### OpenCV ###
The first step is to install [OpenCV 3.4](https://opencv.org/opencv-3-4.html). Use one of the many manuals you find online. Do also install the python wrapper and you should it is better to compile against CUDA if you have a GPU.


### Perceptual Pipeline ###

Install [AR-based tracking](http://wiki.ros.org/ar_track_alvar) (in Ubuntu 16.04 with ROS Kinetic):
```
$ sudo apt-get install ros-melodic-ar-track-alvar
```
Install, moreover, the [libfreenect2](https://github.com/OpenKinect/libfreenect2#debianubuntu-1404). while following the instructions given by the  [IAI Kinect2 project](https://github.com/code-iai/iai_kinect2).
Now we can build the iai_kinect2 package.
```bash
cd ~/catkin_ws/src/perceptual_pipeline/iai_kinect2
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release" --pkg iai_kinect2
```
Do not forgot to include the flag `-Dfreenect2_DIR=$HOME/<pathto>/freenect2/lib/cmake/freenect2 iai_kinect2`. If you installed libfreenect2 in a custom directory.
### anchor_msgs ###
We can now build the anchor_msgs package:
```
catkin_make --pkg anchor_msgs
```

### Anchor Caffe ###
This framework is dependent of the [Caffe Deep learning framework](http://caffe.berkeleyvision.org/), which source code can be found at: [https://github.com/BVLC/caffe](https://github.com/BVLC/caffe)

We first need to install some dependencies.
```bash
$ sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
$ sudo apt-get install --no-install-recommends libboost-all-dev
$ sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
```



`$ sudo apt-get install libopenblas-base libopenblas-dev`


** Build Caffe: **
Clone the repo and then:
```bash
$ cd caffe
$ cp Makefile.config.example Makefile.config
```

** Edit `Makefile.config **

* Set `USE_CUDNN := 1` to enable cudnn accellaration (if available).
* Set `USE_OPENCV := 1` (for OpenCV support).
* Set `BLAS := atlas` to `BLAS := open` for OpenBLAS support.
* Set `OPENCV_VERSION := 3`, if your're using OpenCV >= 3, see [here](https://github.com/BVLC/caffe/issues/3700#issuecomment-187493856)
* Change numpy directory in `PYTHON_INCLUDE` to local install `<numpy_location>/core/include`, e.g. `/usr/local/lib/python2.7/dist-packages/numpy/core/include/`
* Change line `INCLUDE_DIRS := $(PYTHON_INCLUDE) /usr/local/include` to `INCLUDE_DIRS := $(PYTHON_INCLUDE) /usr/local/include /usr/include/hdf5/serial/` if an error with hdf5 occurs.

** Fix hdf5  **
According to [this exchange](https://github.com/NVIDIA/DIGITS/issues/156#issuecomment-114776706), we have to add these symlinks.
```bash
$ cd /usr/lib/x86_64-linux-gnu
$ sudo ln -s libhdf5_serial.so.8.0.2 libhdf5.so
$ sudo ln -s libhdf5_serial_hl.so.8.0.2 libhdf5_hl.so
```
Change this line, as well ([source](https://github.com/NVIDIA/DIGITS/issues/156#issuecomment-219089383)) in the Makefile.config

```bash
$ LIBRARY_DIRS := $(PYTHON_LIB) /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu/hdf5/serial/
```

Now you can build Caffe.
```bash
$ make all -j8
$ make test -j8
$ make runtest -j8
```

The flag `-j8` is here used for here for multicore support (8 cores on intel Core i5). Also, for python support, build:
```bash
$ make pycaffe -j8
```

Create a distribute directory with all the Caffe headers, compiled libraries, binaries, etc.:

`$ make distribute`



Download ReGround GoogLeNet model (fine-tuned with 101 object categories):

```bash
$ roscd anchor_caffe/model
$ ./download_model.py
```

### Anchor Utils ###
This repository contains a library for tools that are used by one or several other packages. More specifically, this package is compiled as a library which, currently, contains a generic database interface for accessing a [MongoDB NoSQL database](https://www.mongodb.com/) for persistent storage of anchors, log data, etc.



This library has been rewriten for the latest C++11 drivers for [MongoDB](http://mongodb.github.io/mongo-cxx-driver/mongocxx-v3/). Latest version of MongoDB Community Edition database, together with all C and C++ drivers, can be installed through the following steps:

* Install the database:
```bash
$ sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 0C49F3730359A14518585931BC711F9BA15703C6
$ echo "deb [ arch=amd64 ] http://repo.mongodb.org/apt/ubuntu trusty/mongodb-org/3.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-3.4.list
$ sudo apt-get update
$ sudo apt-get install -y mongodb-org
```


* Install/update cmake (resion 3.2 is required):
```
$ git clone https://github.com/mongodb/mongo-c-driver.git
$ cd mongo-c-driver
$ git checkout x.y.z  # To build a particular release
$ python build/calc_release_version.py > VERSION_CURRENT
$ mkdir cmake-build
$ cd cmake-build
$ cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF ..
$ make
$ sudo make install
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


### Anchor Reasoning ###

Make sure you got [Distributional Clauses](https://bitbucket.org/problog/dc_problog/src/master/) installed on your machine and also YAP (Yet another Prolog). Make sure, when installing YAP, that you follow the instructions on the Distributional Clauses repository page.

You also need to install the [PyDC](https://github.com/ML-KULeuven/PyDC) python package that wraps around Distributional Clauses.

### Build the remaining packages ###
With all the dependencies installed you a ready to compile the complete project:
```bash
$ cd ~/catkin_ws/src
$ catkin_make anchoring
```

**Note**, to prevent `catkin` from compiling one (or more) package(s), simply add an empty `CATKIN_IGNORE` to the package directory (`CATKIN_IGNORE` is added to `.gitignore`, so all `CATKIN_IGNORE` files will only exist locally). For example, to prevent `catkin` from compiling the **perceptual_pipeline** package:

```bash
$ cd ~/catkin_ws/src/anchoring/perceptual_pipeline
$ touch CATKIN_IGNORE
```
