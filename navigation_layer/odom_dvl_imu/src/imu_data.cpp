#include <ros/ros.h>
#include "imu_data.h"

namespace navigation{

  IMUData::IMUData() : quaternion_(0.0, 0.0, 0.0, 0.0)
    {
        eulerAngle_         = Eigen::Vector3d::Zero();
        angularVelocity_    = Eigen::Vector3d::Zero();
        linearAcceleration_ = Eigen::Vector3d::Zero();

    }
    IMUData::~IMUData() { }

    void IMUData::IMUMsgCallback(sensor_msgs::Imu msg)
    {
        quaternion_.x() = msg.orientation.x;
        quaternion_.y() = msg.orientation.y;
        quaternion_.z() = msg.orientation.z;
        quaternion_.w() = msg.orientation.w;

        // We use the IMU's data sheet transformation because Eigen return yaw on 0-180 basis,
        // which makes it impossible to use for us. This formulae gives yaw on 0-360 and independant from
        // roll pitch yaw
        double q0 = quaternion_.w(), q1 = quaternion_.x(), q2 = quaternion_.y(), q3 = quaternion_.z();
        double m11 = 2 * (q0*q0 - 0.5 + q1*q1);
        double m12 = 2 * (q1*q2 + q0*q3);
        double m13 = 2 * (q1*q3 - q0*q2);
        double m23 = 2 * (q2*q3 + q0*q1);
        double m33 = 2 * (q0*q0 - 0.5 + q3*q3);

        double pitch = std::asin(-m13);
        double roll = std::atan2(m23,m33);
        double yaw = std::atan2(m12, m11) + M_PI;

        yaw = std::fmod((yaw + M_PI), M_PI*2.0);

        eulerAngle_ << roll * RAD_TO_DEGREE, pitch * RAD_TO_DEGREE, yaw * RAD_TO_DEGREE;

        linearAcceleration_ << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;

        angularVelocity_ << msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z;

        SetNewDataReady();
    }

    Eigen::Quaterniond IMUData::GetQuaternion()
    {
        return quaternion_;
    }

    Eigen::Vector3d IMUData::GetOrientation()
    {
        return eulerAngle_;
    }

    Eigen::Vector3d IMUData::GetAngularVelocity()
    {
        return angularVelocity_;
    }
}