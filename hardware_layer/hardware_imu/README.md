# hardware_imu

## Overview
This is a ROS package for interfacing with the [PX4](http://px4.io/) hardware or the [Sparton AHRS-8](https://www.spartonnavex.com/product/ahrs-8/) hardware. 

For the PX4, we are using the in-built inertial measurement unit (IMU) present in the PX4 for the robot.
For the Sparton AHRS8, the driver communicates with the sensor using NMEA protocol and publishes the IMU data as ROS sensor messages.

The `hardware_imu` package has been tested under [ROS](http://www.ros.org) Kinetic and Ubuntu 16.04 LTS. The source code is released under a [BSD 3-Clause license](LICENSE.md).


## Setting up hardware_imu

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- Following ROS Packages: [mavros](http://wiki.ros.org/mavros), [mavros_extras](http://wiki.ros.org/mavros_extras)

According to the installation instructions present [here](https://github.com/mavlink/mavros/blob/master/mavros/README.md#binary-installation-deb), run the following commands to install GeographicLib datasets which is required as a mavros dependency:
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

### Calibrating the sensor

Download [QGround Control](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html) and run it with PIXHAWK attached with your computer. Read the manual [here](https://docs.qgroundcontrol.com/en/SetupView/Sensors.html) for further instructions.

### Building the package
Run the following command from the root of your catkin workspace:
```
catkin_make --pkg hardware_imu
```

## Usage
To run the **PIXHAWK IMU**:
```
roslaunch hardware_imu imu_px4.launch
```
where the PIXHAWK is assumed to be connected at the `/dev/ttyACM0` port.

To run the **Sparton AHRS8 IMU**:

1. Check the serial port to which the sensor is connected at and change the device path in the [launch file](launch/ahrs-8.launch)
```bash
# to verify if device is at /dev/ttyUSB0, run:
udevadm info -a -p  $(udevadm info -q path -n /dev/ttyUSB0)
```
2. Ensure that python cript has executable permission:
```bash
chmod +x sparton_ahrs8_driver/scripts/ahrs8_nmea.py
```
3. Run the launch file:
```bash
roslaunch sparton_ahrs8_driver ahrs-8.launch
```

## Nodes

### mavros_node

The `mavros_node` is the main communication node for the MAVLink communication protocol used in the PX4 autopilot. The node belongs to the [mavros](http://wiki.ros.org/mavros) package.

#### Published Topics

* **`~imu/data`** ([sensor_msgs/Imu])
  Imu data, orientation computed by FCU
* **`~imu/data_raw`** ([sensor_msgs/Imu])
  Raw IMU data without orientation
* **`~imu/mag`** ([sensor_msgs/MagneticField])
  FCU compass data
* **`~imu/temperature`** ([sensor_msgs/Temperature])
  Temperature reported by FCU (usually from barometer)
* **`~imu/atm_pressure`** ([sensor_msgs/FluidPressure])
  Air pressure.

### Parameters
* **`~imu/frame_id`** (string, default: fcu)
  Frame ID for this plugin.
* **`~imu/linear_acceleration_stdev`** (double, default: 0.0003)
  Gyro's standard deviation
* **`~imu/angular_velocity_stdev`** (double, default: !dec 0.02)
  Accel's standard deviation
* **`~imu/orientation_stdev`** (double, default: 1.0)
  Standard deviation for IMU.orientation
* **`~imu/magnetic_stdev`** (double, default: 0.0)
  Standard deviation for magnetic field message (undefined if: 0.0)

### mavros.cpp

This node takes the IMU data in quateranion format and convert it into euler angles. We calculate the roll, pitch and yaw using the following formulas:

```
yaw = atan2 ( 2 * x * y - 2 * w * z, 2 * w * w + 2 * x * x - 1)
roll = atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1)
pitch = -asin(2 * q1 * q3 + 2 * q0 * q2)

```
where `q = (w, x, y, z)` is the quaternion angle.

#### Subscribed Topics

* **`~/mavros/imu/data`** ([sensor_msgs/Imu])
  Imu data, orientation computed by FCU

#### Published Topics

* **`/mavros/imu/yaw`** ([std_msgs/Float64])
  Yaw data computed using quaternion to euler angles transformation
  **`/mavros/imu/pitch`** ([std_msgs/Float64])
  Pitch data computed using quaternion to euler angles transformation
  **`/mavros/imu/roll`** ([std_msgs/Float64])
  Roll data computed using quaternion to euler angles transformation


### ahrs8_nmea.py

The node communicate with the sensor using the NMEA protocol and publishes IMU data.

#### Parameters
* **`~frame_id`** (string, default: `ahrs8_imu`)
  Frame ID for this plugin
* **`~port`** (string, default: `/dev/ttyUSB0`)
  Port at which sensor is connected
* **`~baud`** (double, default: `115200`)
  Baud rate for communication with the sensor

#### Published Topics

* **`~imu/data`** ([sensor_msgs/Imu])
  IMU orientation data, orientation in the `ahrs8_imu` frame

### sparton.cpp

This node takes the IMU data in quateranion format and convert it into euler angles. We calculate the roll, pitch and yaw using the above formulas again

#### Subscribed Topics

* **`~/sparton/imu/data`** ([sensor_msgs/Imu])
  Imu data, orientation computed by FCU

#### Published Topics

* **`/sparton/imu/yaw`** ([std_msgs/Float64])
  Yaw data computed using quaternion to euler angles transformation
  **`/sparton/imu/pitch`** ([std_msgs/Float64])
  Pitch data computed using quaternion to euler angles transformation
  **`/sparton/imu/roll`** ([std_msgs/Float64])
  Roll data computed using quaternion to euler angles transformation   

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AUV-IITK/fourtran/issues).

[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
[sensor_msgs/MagneticField]: http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html
[sensor_msgs/FluidPressure]: http://docs.ros.org/api/sensor_msgs/html/msg/FluidPressure.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
[std_msgs/Float64]: http://docs.ros.org/lunar/api/std_msgs/html/msg/Float64.html
