# hardware_arduino

## Overview

This is a ROS package for interacting arduino through the [rosserial_arduino](http://wiki.ros.org/rosserial_arduino) package. The package is meant to actuate thrusters connected to the arduino through a motor driver, and publish the measurements taken from various sensors connected to the arduino onto separate topics.

The `hardware_arduino` package has been tested under [ROS](http://www.ros.org) Kinetic and Ubuntu 16.04 LTS. The source code is released under a [BSD 3-Clause license](LICENSE.md).

The hardware used are as follows:
* [Arduino MEGA](https://store.arduino.cc/usa/arduino-mega-2560-rev3)
* [8V-28V, 5Amp Dual DC Motor Driver with Current Sensor](http://www.nex-robotics.com/products/motor-drivers/8v-28v-5amp-dual-dc-motor-driver-with-current-sense.html)
* [Pressure Sensor](https://www.bluerobotics.com/store/electronics/bar30-sensor-r1/)


## Setting up Arduino

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- Following ROS Packages: [rosserial](http://wiki.ros.org/rosserial), [rosserial_arduino](http://wiki.ros.org/rosserial_arduino)

### Setting up the Udev rules for arduino

Run the following command to copy the udev rules specified for the Arduino port:
```
cd ~/catkin_ws/src/auv2018/utils
sudo bash clone_udev.sh
```

__NOTE:__ This connects the arduino always to the port named `/dev/arduino`.

### Preparing the Serial Port
Arduino will likely connect to computer as port `/dev/arduino`. The easiest way to make the determination is to unplug all other USB devices, plug in your Arduino, then run the command:
```
ls /dev*
```

Next make sure you have read/write access to the port. Assuming your Arduino is connected on `/dev/arduino`, run the command:

```
ls -l /dev/arduino
```
You should see an output similar to the following:
```
crw-rw---- 1 root dialout 166, 0 2018-03-05 08:31 /dev/arduino
```

Note that only root and the "dialout" group have read/write access. Therefore, you need to be a member of the dialout group. You only have to do this once and it should then work for all USB devices you plug in later on.

To add yourself to the dialout group, run the command:
```
sudo usermod -a -G dialout your_user_name
```
where `your_user_name` is your Linux login name. You will likely have to log out of your X-window session then log in again, or simply reboot your machine if you want to be sure.

When you log back in again, try the command:
```
groups
```
and you should see a list of groups you belong to including dialout.

### Building Arduino code

Run the following command:
```
cd ~/catkin_ws
catkin_make --pkg hardware_arduino
```

### Uploading code to Arduino
Run the following command:
```
cd ~/catkin_ws
source devel/setup.zsh
catkin_make hardware_arduino_firmware_arduino_node    
catkin_make hardware_arduino_firmware_arduino_node-upload     
```

## Robot Orientation

```
          FRONT
          -----
          SWAY1
          HEAVE1
            -
  SURGE1           SURGE2
            -
          HEAVE2
          SWAY2
          -----
          BACK
```

For technical definition of terms, refer to documentation [here](https://en.wikipedia.org/wiki/Ship_motions).

## Usage

To connect to the arduino, run:
```
roslaunch hardware_arduino hardware_arduino.launch
```
First upload the code on arduino through Arduino IDE.

## Nodes

### arduino_node
Subscribes to topics with PWM data and actuate the thrusters with that duty cycle, and also publishes the data obtained from pressure sensor

__NOTE:__ Pins configurations are specified in the [ArduinoConfig.h](include/ArduinoConfig.h) file.

#### Subscribed Topics
* **`/thruster/surge1/pwm`** ([std_msgs/Int32])
* **`/thruster/surge2/pwm`** ([std_msgs/Int32])
* **`/thruster/heave1/pwm`** ([std_msgs/Int32])
* **`/thruster/heave2/pwm`** ([std_msgs/Int32])
* **`/thruster/sway1/pwm`** ([std_msgs/Int32])
* **`/thruster/sway1/pwm`** ([std_msgs/Int32])

#### Published Topics
* **`/pressure_sensor/pressure`** ([underwater_sensor_msgs/Pressure]): Pressure sensor data (in Pascals)


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AUV-IITK/auv2017/issues).

[std_msgs/Int32]: http://docs.ros.org/api/std_msgs/html/msg/Int32.html
[underwater_sensor_msgs/Pressure]: http://docs.ros.org/hydro/api/underwater_sensor_msgs/html/msg/Pressure.html
