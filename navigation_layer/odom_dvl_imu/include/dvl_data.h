#ifndef DVL_DATA_H
#define DVL_DATA_H

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <eigen3/Eigen/Geometry>
#include "navigation_device.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>

namespace navigation{

    class DvlData: public NavigationDevice 
    {
        typedef void (DvlData::*IntegrationMethodT) (const double &);
    
    public:
    
        const double BAR_TO_METER_OF_WATER = 10.1972;

        enum IntegrationMethodType
        {
            StdMethod  = 0,
            RKMethod,
            DefaultMethod
        };

        DvlData(IntegrationMethodType integrationMethodType = RKMethod);
        ~DvlData();

        void DvlTwistCallback(geometry_msgs::TwistWithCovarianceStamped msg);
        void DvlPressureCallback(std_msgs::Float32 msg);

        std::vector<double> x_vel;
        std::vector<double> y_vel;
        std::vector<double> z_vel;
        int x_count = 0;
        int y_count = 0;
        int z_count = 0;
        int vel_count = 0;

        Eigen::Vector3d GetPositionXYZ();
        Eigen::Vector3d GetVelocityXYZ();
        double GetPositionZFromPressure();
        std_msgs::Float32 GetPressure();

    private:
        void StdIntegrationMethod(const double &dt_sec);
        void RKIntegrationMethod(const double &dt_sec);
        double Average(std::vector<double> array);
        bool inRange (double x, double avg, double thres);

        ros::Time last_timestamp_;

        Eigen::Vector3d positionIncrement_;
        Eigen::MatrixXd historyPositionIncrement_;

        geometry_msgs::TwistWithCovarianceStamped dvl_twist_;
        std_msgs::Float32 dvl_pressure_;

        IntegrationMethodT integrationMethod_;
    };

}
#endif // DVLDATA_H