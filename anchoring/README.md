# The Anchoring Management System #

This repository contains the ROS packages that constitutes the anchoring management system. This system is responsible for the long-term management of anchors through the three anchoring functions: acquire, re-acquire and track.

## Dependencies ##

The anchoring management system uses a [MongoDB NoSQL database](https://www.mongodb.com/) for persistent storage of anchors. 
MongoDB can simply be installed in Ubuntu (version 14.04 >) through the use of the standard package manager:

```
$ sudo apt-get install mongodb libmongo-client-dev
```


