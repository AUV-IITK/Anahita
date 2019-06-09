#include "mapping_main.h"

namespace mapping{

  void MappingNode::transform_broadcaster_initial(tf::TransformBroadcaster& map_to_odom_broadcaster)
  {
      ROS_INFO("Publishing a transform");
      geometry_msgs::TransformStamped map_to_odom_trans;
      map_to_odom_trans.header.stamp = ros::Time::now();
      map_to_odom_trans.header.frame_id = "map";
      map_to_odom_trans.child_frame_id = "odom";

      map_to_odom_trans.transform.translation.x = 0;
      map_to_odom_trans.transform.translation.y = 0;
      map_to_odom_trans.transform.translation.z = 0;
      map_to_odom_trans.transform.rotation.w = 1;
      map_to_odom_trans.transform.rotation.x = 0;
      map_to_odom_trans.transform.rotation.y = 0;
      map_to_odom_trans.transform.rotation.z = 0;

        // send the transform
      map_to_odom_broadcaster.sendTransform(map_to_odom_trans);
      ROS_INFO("Published the transform");
  }

  MappingNode::MappingNode(const ros::NodeHandlePtr &nh) : nh_(nh), _map({"elevation", "occupancy"})
  {
    _map.setFrameId("map");
    _map.setGeometry(grid_map::Length(30, 30), 1);

      map_publisher = nh_->advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
      odom_subscriber = nh_->subscribe("/anahita/pose_gt/relay", 10, &MappingNode::odometry_update_cb, this);
      ROS_INFO("Created map with size %f x %f m (%i x %i cells).", _map.getLength().x(), _map.getLength().y(), _map.getSize()(0), _map.getSize()(1));

    for (grid_map::GridMapIterator it(_map); !it.isPastEnd(); ++it) {
          grid_map::Position position;
          _map.getPosition(*it, position);
          _map.at("elevation", *it) = 0;
          _map.at("occupancy", *it) = 0;
      }

    loadMapFromYAML(_map);
  }

  void MappingNode::odometry_update_cb(const nav_msgs::Odometry::ConstPtr& msg)
  {
      // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
      // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

      grid_map::Position checked_space(msg->pose.pose.position.x,msg->pose.pose.position.y);
      
      //need to check if already visited, or object is present here
      if(_map.atPosition("occupancy", checked_space) == 0)
          _map.atPosition("occupancy", checked_space) = -1;
        
      ROS_INFO("We have already gone to: %d %d",msg->pose.pose.position.x,msg->pose.pose.position.y);
      
  }

  void MappingNode::loadMapFromYAML(grid_map::GridMap &map)
  {
      YAML::Node yaml_map = YAML::LoadFile(map_yaml_location);
      ROS_INFO("Loaded: %d", yaml_map["map"]["gate"]["x"].as<int>());
      grid_map::Position gate_position(yaml_map["map"]["gate"]["x"].as<int>(),yaml_map["map"]["gate"]["y"].as<int>());
      grid_map::Position torpedo_position(yaml_map["map"]["torpedo"]["x"].as<int>(),yaml_map["map"]["torpedo"]["y"].as<int>());
      _map.atPosition("occupancy", gate_position) = 1;
      _map.atPosition("occupancy", torpedo_position) = 1;
      ROS_INFO("Loaded map file");
  }

  void MappingNode::PublishMap()
  {
      ros::Time time = ros::Time::now();
      // Publish grid map.
      _map.setTimestamp(time.toNSec());
      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(_map, message);
      map_publisher.publish(message);
      ROS_INFO("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
      // Wait for next cycle.
  }

  void MappingNode::Spin()
  {
      ros::Rate loop_rate(15); // 100 hz
      while (ros::ok())
      {
          ros::spinOnce();
          loop_rate.sleep();
      }
  }
}