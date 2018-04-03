
# AUV Hyperion

This repository contains the code for our second underwater vehicle `Hyperion`. 


| S. No. 	| Operating System 	|  ROS Version 	| Build Status 	|
|:------:	|:----------------:	|:------------:	|:------------:	|
| 1.     	| [Ubuntu 16.04 LTS](http://releases.ubuntu.com/16.04/) 	| [Kinetic Kame](http://wiki.ros.org/kinetic) 	|  [![Build Status](https://travis-ci.org/AUV-IITK/auv2018.svg?branch=master)](https://travis-ci.org/AUV-IITK/auv2018)|

## How to build the repository?

1. Create a catkin worspace following the guidelines given [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
```

2. Clone this repository to your catkin workspace
```bash
cd ~/catkin_ws/src
git clone https://github.com/AUV-IITK/auv2018.git
```

3. Install all dependency packages to run the repository

The repository requires the following ROS packages to run: [usb_cam](http://wiki.ros.org/usb_cam), [geographic_msgs](http://wiki.ros.org/geometry_msgs), [rosserial_arduino](http://wiki.ros.org/rosserial_arduino), [underwater_sensor_msgs](http://wiki.ros.org/underwater_sensor_msgs). You can build and install those packages from their respective sources or you can use the following command in Ubuntu 16.04 to install them. *If you are building from source or using a different package manager, make sure you are building the kinetic version of these packages to ensure maximum compatibility.*
```bash
sudo apt-get install ros-kinetic-usb-cam ros-kinetic-geographic-msgs ros-kinetic-rosserial-arduino ros-kinetic-underwater-sensor-msgs
```

4. Build the package using [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make)
```bash
cd ~/catkin_ws
# To maximize performance, build the workspace in Release mode
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Contribution Guidelines

To get started with contributing to this repository, look out for open issues [here](https://github.com/AUV-IITK/auv2018/issues). Kindly read the [__Developer's Guide__](https://github.com/AUV-IITK/AUVWiki/wiki/Developers-Guide) before sending a pull request! :)
