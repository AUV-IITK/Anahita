#ifndef DVL_CONNECTION_H
#define DVL_CONNECTION_H

#include <string>
#include <ros/node_handle.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include "dvl_ethernet.h"
#include "dvl_data.h"

namespace hardware_dvl {

    class DVLNode {
        public:

            DVLNode(const ros::NodeHandlePtr &nh);
            ~DVLNode();
            void Spin();

        private:
    
            void FillTwistMessage(ros::Time timestamp);
            void FillFluidPressureMessage(ros::Time timestamp);

            ros::NodeHandlePtr nh_;
            DVLEthernet socket_;

            DVLformat21_t dvl_data_;

            ros::Time timestamp_;
            ros::Publisher dvl_twist_publisher_;
            ros::Publisher dvl_fluid_pressure_publisher_;
    };

}

#endif //DVL_NODE_H