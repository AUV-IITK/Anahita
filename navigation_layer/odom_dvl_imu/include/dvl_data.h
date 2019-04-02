#ifndef DVL_DATA_H
#define DVL_DATA_H

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/FluidPressure.h>
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
        void DvlPressureCallback(sensor_msgs::FluidPressure msg);


        Eigen::Vector3d GetPositionXYZ();
        Eigen::Vector3d GetVelocityXYZ();
        double GetPositionZFromPressure();
        sensor_msgs::FluidPressure GetPressure();

    private:
        void StdIntegrationMethod(const double &dt_sec);
        void RKIntegrationMethod(const double &dt_sec);

        ros::Time last_timestamp_;

        Eigen::Vector3d positionIncrement_;
        Eigen::MatrixXd historyPositionIncrement_;

        geometry_msgs::TwistWithCovarianceStamped dvl_twist_;
        sensor_msgs::FluidPressure dvl_pressure_;

        IntegrationMethodT integrationMethod_;
    };

}
#endif // DVLDATA_H