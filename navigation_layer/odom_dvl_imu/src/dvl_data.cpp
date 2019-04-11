#include <ros/ros.h>
#include "dvl_data.h"

namespace navigation{

    DvlData::DvlData(IntegrationMethodType integrationMethodType) :
            last_timestamp_(ros::Time::now())
    {
        positionIncrement_        = Eigen::Vector3d::Zero();
        historyPositionIncrement_ = Eigen::MatrixXd::Zero(3, 4);

        switch (integrationMethodType)
        {
            case StdMethod :
                integrationMethod_ = &DvlData::StdIntegrationMethod;
                break;
            case RKMethod  :
                integrationMethod_ = &DvlData::RKIntegrationMethod;
                break;
            default :
                integrationMethod_ = &DvlData::StdIntegrationMethod;
                break;
        }
    }

    DvlData::~DvlData() { }
    void DvlData::DvlTwistCallback(geometry_msgs::TwistWithCovarianceStamped msg)
    {
        // dvl_twist_ = msg;
        dvl_twist_.header = msg.header;
        dvl_twist_.twist = msg.twist.twist;
        dvl_twist_.twist.linear.x = msg.twist.twist.linear.z;
        dvl_twist_.twist.linear.z = -msg.twist.twist.linear.x;

        SetNewDataReady();
    }

    void DvlData::DvlPressureCallback(sensor_msgs::FluidPressure msg)
    {
        dvl_pressure_ = msg;
        SetNewDataReady();
    }

    Eigen::Vector3d DvlData::GetPositionXYZ()
    {
        ros::Duration dt = ros::Time::now() - last_timestamp_;
        double dt_sec = dt.toSec();

        (this->*integrationMethod_)(dt_sec);

        last_timestamp_ = ros::Time::now();

        return positionIncrement_;
    }

    void DvlData::StdIntegrationMethod(const double &dt_sec)
    {
        positionIncrement_ << dvl_twist_.twist.twist.linear.x * dt_sec, dvl_twist_.twist.twist.linear.y * dt_sec, dvl_twist_.twist.twist.linear.z * dt_sec;
    }

    void DvlData::RKIntegrationMethod(const double &dt_sec)
    {
        positionIncrement_ << dvl_twist_.twist.twist.linear.x * dt_sec, dvl_twist_.twist.twist.linear.y * dt_sec, dvl_twist_.twist.twist.linear.z * dt_sec;
        historyPositionIncrement_.block<3,3>(0, 1) = historyPositionIncrement_.block<3,3>(0, 0);
        historyPositionIncrement_.col(0) = positionIncrement_;
        positionIncrement_ = (1.0 / 6.0) * (historyPositionIncrement_.col(0) + 2 * historyPositionIncrement_.col(1) + 2 * historyPositionIncrement_.col(2) + historyPositionIncrement_.col(3));
    }

    Eigen::Vector3d DvlData::GetVelocityXYZ()
    {
        Eigen::Vector3d twist;
        twist << dvl_twist_.twist.twist.linear.x, dvl_twist_.twist.twist.linear.y, dvl_twist_.twist.twist.linear.z;
        return twist;
    }

    sensor_msgs::FluidPressure DvlData::GetPressure()
    {
        return dvl_pressure_;
    }

    double DvlData::GetPositionZFromPressure()
    {
        return (101.325 - dvl_pressure_.fluid_pressure) / 9.80638;
    }
}