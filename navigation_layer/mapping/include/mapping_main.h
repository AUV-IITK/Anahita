#ifndef MAPPING_MAIN_H_
#define MAPPING_MAIN_H_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include "yaml-cpp/yaml.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include<string>
#include "nav_msgs/Odometry.h"

namespace mapping
{

class MappingNode
{
public:
    MappingNode(const ros::NodeHandlePtr &nh);
    ~MappingNode();

    void Spin();
private:
    ros::NodeHandlePtr nh_;
    
    ros::Subscriber odom_subscriber;
    ros::Publisher map_publisher;
    grid_map::GridMap _map({"elevation", "occupancy"});

    tf::TransformBroadcaster map_to_odom_broadcaster;
    
    _map.setFrameId("map");
    _map.setGeometry(Length(30,30), 1);

    std::string map_yaml_location = "/home/ayush/Projects/anahita_ws/src/Anahita/navigation_layer/mapping/config/transdec.yaml";
    /*
    ros::Subscriber dvlTwistSubscriber_;
    ros::Subscriber dvlPressureSubscriber_;
    ros::Subscriber imuSubscriber_;

    ros::Publisher navigationOdomPublisher_;
    tf::TransformBroadcaster odom_broadcaster;

    ros::ServiceServer navigationDepthOffsetServer_;
    ros::ServiceServer navigationXYOffsetServer_;

    DvlData dvlData_;
    IMUData imuData_;

    ExtendedKalmanFilter dvlFilter_;
    Eigen::Vector3d poseEstimation_;

    double zOffset_;
    double positionFromDepth_;

    Eigen::Quaterniond quaternion_;
    Eigen::Vector3d position_;
    Eigen::Vector3d incrementPosition_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d angularVelocity_;
    Eigen::Vector3d eulerAngel_;
    */
};

} // namespace mapping
#endif //MAPPING_NODE_H_