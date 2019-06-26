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

    void transform_broadcaster_initial(tf::TransformBroadcaster& map_to_odom_broadcaster);
    void odometry_update_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void loadMapFromYAML(grid_map::GridMap &map);
    void PublishMap();
    void Spin();

private:
    ros::NodeHandlePtr nh_;
    
    ros::Subscriber odom_subscriber;
    ros::Publisher map_publisher;
    grid_map::GridMap _map;

    tf::TransformBroadcaster map_to_odom_broadcaster;
    
    std::string map_yaml_location = "/home/ayush/Projects/anahita_ws/src/Anahita/navigation_layer/mapping/config/transdec.yaml";

};

} // namespace mapping
#endif //MAPPING_NODE_H_