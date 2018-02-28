If you are using OpenCV3, you might have to modify line 54 from:

```
set(EXPORTED_DEPENDENCIES OpenCL)
```
to
```
set(EXPORTED_DEPENDENCIES OpenCL)
add_definitions( -fexceptions )
```
in `ai_kinect2/kinect2_registration/CMakeLists.txt`


## Dependencies ##

There are a LOT of different dependencies for all the different sub-packages, and this README file will be updated...  eventually.  

* Install [AR-based tracking](http://wiki.ros.org/ar_track_alvar) (in Ubuntu 16.04 with ROS Kinetic):
```
$ sudo apt-get install ros-kinetic-ar-track-alvar
```
