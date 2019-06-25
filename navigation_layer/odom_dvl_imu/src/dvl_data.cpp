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
        dvl_twist_ = msg;
        ROS_INFO("Inside callback");

        
        if(dvl_twist_.twist.twist.linear.x != -32.768 && dvl_twist_.twist.twist.linear.y != -32.768 && dvl_twist_.twist.twist.linear.z != -32.768)
        {
            dvl_twist_.twist.twist.linear.x = msg.twist.twist.linear.y;
            dvl_twist_.twist.twist.linear.y = -msg.twist.twist.linear.x;
            dvl_twist_.twist.twist.linear.z = msg.twist.twist.linear.z;
        }
        else
        {
            ROS_ERROR("--------- Error reading from DVL ------------- ");
            dvl_twist_.twist.twist.linear.x = 0;
            dvl_twist_.twist.twist.linear.z = 0;
            dvl_twist_.twist.twist.linear.y = 0;
        }

        /*
        double x_vel_new = dvl_twist_.twist.twist.linear.x;
        double y_vel_new = dvl_twist_.twist.twist.linear.y;
        double z_vel_new = dvl_twist_.twist.twist.linear.z;
        if(vel_count < 10){
            x_vel.push_back(x_vel_new);
            y_vel.push_back(y_vel_new);
            z_vel.push_back(z_vel_new);
            vel_count++;
        }
        else
        {
            if (inRange(x_vel_new, Average(x_vel), 2) && !(x_vel_new!=-32.768)) {
                std::rotate (x_vel.begin(), x_vel.begin() + 1, x_vel.end());
                x_vel[9] = x_vel_new;

            }
            if (inRange(y_vel_new, Average(y_vel), 2) && !(y_vel_new!=-32.768)) {
                std::rotate (y_vel.begin(), y_vel.begin() + 1, y_vel.end());
                y_vel[9] = y_vel_new;
            }
            if (inRange(z_vel_new, Average(z_vel), 2) && !(z_vel_new!=-32.768)) {
                std::rotate (z_vel.begin(), z_vel.begin() + 1, z_vel.end());
                z_vel[9] = z_vel_new;
            }
        }
        dvl_twist_.twist.twist.linear.x = Average (x_vel);
        dvl_twist_.twist.twist.linear.y = Average (y_vel);
        dvl_twist_.twist.twist.linear.z = Average (z_vel);
*/
        SetNewDataReady();
    }

    void DvlData::DvlPressureCallback(std_msgs::Float32 msg)
    {
        dvl_pressure_ = msg;
        dvl_pressure_.data = -msg.data;
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

    std_msgs::Float32 DvlData::GetPressure()
    {
        return dvl_pressure_;
    }

    double DvlData::GetPositionZFromPressure()
    {
        return dvl_pressure_.data;
    }
}
