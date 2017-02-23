# Anchor Utils #

This repository contains a library for tools that are used by one or several other packages. More specifically, this package is compiled as a library which, currently, contains the following tools:

* **database**: a generic database interface for accessing a [MongoDB NoSQL database](https://www.mongodb.com/) for persistent storage of anchors, log data, etc.


## Dependencies ##

This library has been rewriten for the latest C++11 drivers for [MongoDB](http://mongodb.github.io/mongo-cxx-driver/mongocxx-v3/). Latest version of MongoDB Community Edition database, together with all C and C++ drivers, can be installed through the following steps (tested in Ubuntu 14.04):

* Install the database:
```
$ sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 0C49F3730359A14518585931BC711F9BA15703C6
$ echo "deb [ arch=amd64 ] http://repo.mongodb.org/apt/ubuntu trusty/mongodb-org/3.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-3.4.list
$ sudo apt-get update
$ sudo apt-get install -y mongodb-org
```
*Note, instructions above is for Ubuntu 14.04, for other versions, see [official install instruction](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/).*

* Install/update cmake (resion 3.2 is required):
```
$ sudo apt-get install software-properties-common 
$ sudo add-apt-repository ppa:george-edison55/cmake-3.x
$ sudo apt-get update
$ sudo apt-get install cmake
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
*Note, in later Ubuntu versions can the instructions above be substituted by:* `$ sudo apt-get install libmongoc-1.0-0`

* Install MongoDB C++ drivers:
```
$ git clone https://github.com/mongodb/mongo-cxx-driver.git --branch releases/stable --depth 1
$ cd mongo-cxx-driver/build
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
$ sudo make EP_mnmlstc_core # For MNMLSTC polyfill
$ make && sudo make install
```
*Note, make sure that library path of the install libraries, e.g.* `/usr/local/lib`*, is part of your* `LD_LIBRARY_PAHT`.

