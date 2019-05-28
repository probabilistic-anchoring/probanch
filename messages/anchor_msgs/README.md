# Anchor Messages #

A dedicated package for all anchor specific ROS messages. If messages are changed, and because of various nested dependencies, is it advisable to compile this package prior other packages:

```
$ catkin_make --pkg anchor_msgs
$ catkin_make
```
