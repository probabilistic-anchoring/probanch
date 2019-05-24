# The Anchoring Caffe framework wrapper #

This repository contains a ROS friendly wrapper packages for accesing the Caffe framework (for image classification).


## Dependencies ##

This framework is dependent of the [Caffe Deep learning framework](http://caffe.berkeleyvision.org/), which source code can be found at: [https://github.com/BVLC/caffe](https://github.com/BVLC/caffe)

### Ubuntu 14.04 dependencies: ###

```bash
$ sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
$ sudo apt-get install --no-install-recommends libboost-all-dev
$ sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
```

### OpenBLAS support (default is ATLAS used): ###

`$ sudo apt-get install libopenblas-base libopenblas-dev`


### Build Caffe: ###

```bash
$ roscd anchor_caffe/caffe
$ cp Makefile.config.example Makefile.config
```

### Edit `Makefile.config` ###

* Set `USE_CUDNN := 1` to enable cudnn accellaration (if available). 
* Set `USE_OPENCV := 1` (for OpenCV support). 
* Set `BLAS := atlas` to `BLAS := open` for OpenBLAS support.
* Set `OPENCV_VERSION := 3`, if your're using OpenCV >= 3, see [here](https://github.com/BVLC/caffe/issues/3700#issuecomment-187493856)
* Change numpy directory in `PYTHON_INCLUDE` to local install `<numpy_location>/core/include`, e.g. `/usr/local/lib/python2.7/dist-packages/numpy/core/include/` 
* Change line `INCLUDE_DIRS := $(PYTHON_INCLUDE) /usr/local/include` to `INCLUDE_DIRS := $(PYTHON_INCLUDE) /usr/local/include /usr/include/hdf5/serial/` if an error with hdf5 occurs.

### Fix hdf5 14.04/16.04 ###
According to [this exchange](https://github.com/NVIDIA/DIGITS/issues/156#issuecomment-114776706), we have to add these symlinks, because later hdf5 inclusion will fail with

```bash
$ /usr/bin/ld: cannot find -lhdf5_hl
$ /usr/bin/ld: cannot find -lhdf5
```

To fix, create these symlinks

```bash
$ cd /usr/lib/x86_64-linux-gnu
$ sudo ln -s libhdf5_serial.so.8.0.2 libhdf5.so
$ sudo ln -s libhdf5_serial_hl.so.8.0.2 libhdf5_hl.so
```

In return, add `hdf5` includes and libraries to these lines in `Makefile.config` (according to: [source](https://github.com/NVIDIA/DIGITS/issues/156#issuecomment-219089383)):

```bash
INCLUDE_DIRS := $(PYTHON_INCLUDE) /usr/local/include /usr/include/hdf5/serial/
LIBRARY_DIRS := $(PYTHON_LIB) /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu/hdf5/serial/
```

**NOTE**: `PYTHON_PATH` should be custom, to accomodate for additional packages of OpenCV, set in `<your_shell>rc`

```bash
$ echo 'export PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH' >> ~/<your_shell>rc
```

`$ make all -j8; make test -j8; make runtest -j8`

The flag `-j8` is here used for here for multicore support (8 cores on inte l Core i5). Also, for python support, build: `$ make pycaffe -j8`

Create a distribute directory with all the Caffe headers, compiled libraries, binaries, etc.:

`$ make distribute`


* Download ReGround GoogLeNet model (fine-tuned with 101 object categories):

```bash
$ roscd anchor_caffe/model
$ ./download_model.py
```

*Note, the download script is dependent of the `requests` Python library, which can be installed by:* `$ sudo pip install requests`


