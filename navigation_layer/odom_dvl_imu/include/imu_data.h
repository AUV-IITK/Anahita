#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <sensor_msgs/Imu.h>
#include "navigation_device.h"
#include <eigen3/Eigen/Geometry>
namespace navigation{

class IMUData: public NavigationDevice {
public:
    const double RAD_TO_DEGREE = 180.0f / M_PI;

    IMUData();
    ~IMUData();

    void IMUMsgCallback(sensor_msgs::Imu msg);
    Eigen::Quaterniond GetQuaternion();
    Eigen::Vector3d    GetOrientation();
    Eigen::Vector3d    GetAngularVelocity();

private:

    Eigen::Quaterniond quaternion_;
    Eigen::Vector3d    eulerAngle_;
    Eigen::Vector3d    angularVelocity_;
    Eigen::Vector3d    linearAcceleration_;
};

}
#endif // IMUDATA_H