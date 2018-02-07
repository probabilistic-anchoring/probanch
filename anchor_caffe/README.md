# The Anchoring Caffe framework wrapper #

This repository contains a ROS friendly wrapper packages for accesing the Caffe framework (for image classification).


## Dependencies ##

This framework is dependent of the [Caffe Deep learning framework](http://caffe.berkeleyvision.org/), which source code can be found at: [https://github.com/BVLC/caffe](https://github.com/BVLC/caffe)

* Ubuntu 14.04 dependencies:

```
$ sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
$ sudo apt-get install --no-install-recommends libboost-all-dev
$ sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
```

* OpenBLAS support (default is ATLAS used):

`$ sudo apt-get install libopenblas-base libopenblas-dev`


* Build Caffe:

```
$ roscd anchor_caffe/caffe
$ cp Makefile.config.example Makefile.config
```

Edit `Makefile.config` and uncomment `CPU_ONLY := 1` (for CPU-only Caffe) and set `USE_OPENCV := 1` (for OpenCV support). Also, change `BLAS := atlas` to `BLAS := open` for OpenBLAS support.

`$ make all -j8; make test -j8; make runtest -j8`

The flag `-j8` is here used for here for multicore support (8 cores on inte l Core i5). Also, for python support, build: `$ make pycaffe -j8`

Create a distribute directory with all the Caffe headers, compiled libraries, binaries, etc.:

`$ make distribute`


* Download ReGround GoogLeNet model (fine-tuned with 101 object categories):

```
$ roscd anchor_caffe/model
$ ./download_model.py
```

*Note, the download script is dependent of the `requests` Python library, which can be installed by:* `$ sudo pip install requests`


