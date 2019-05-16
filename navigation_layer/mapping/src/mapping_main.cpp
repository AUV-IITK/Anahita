#include "mapping_main.h"

using namespace grid_map;
GridMap map({"elevation", "occupancy"});

std::string map_yaml_location = "/home/ayush/Projects/anahita_ws/src/Anahita/navigation_layer/mapping/config/transdec.yaml";

void transform_broadcaster_initial(tf::TransformBroadcaster& map_to_odom_broadcaster)
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

void odometry_update_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

    Position checked_space(msg->pose.pose.position.x,msg->pose.pose.position.y);
    
    //need to check if already visited, or object is present here
    if(map.atPosition("occupancy", checked_space) == 0)
        map.atPosition("occupancy", checked_space) = -1;
      
    ROS_INFO("We have already gone to: %d %d",msg->pose.pose.position.x,msg->pose.pose.position.y);
    
}

void loadMapFromYAML(GridMap &map)
{
    YAML::Node yaml_map = YAML::LoadFile(map_yaml_location);
    ROS_INFO("Loaded: %d", yaml_map["map"]["gate"]["x"].as<int>());
    Position gate_position(yaml_map["map"]["gate"]["x"].as<int>(),yaml_map["map"]["gate"]["y"].as<int>());
    Position torpedo_position(yaml_map["map"]["torpedo"]["x"].as<int>(),yaml_map["map"]["torpedo"]["y"].as<int>());
    map.atPosition("occupancy", gate_position) = 1;
    map.atPosition("occupancy", torpedo_position) = 1;
    ROS_INFO("Loaded map file");
}

int main(int argc, char** argv)
{
  ROS_INFO("Starting the mapping node");
  // Initialize node and publisher.
  ros::init(argc, argv, "map_publisher_node");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
  ros::Subscriber odom_subscriber = nh.subscribe("/anahita/pose_gt", 10, odometry_update_cb);
  tf::TransformBroadcaster map_to_odom_broadcaster;
  transform_broadcaster_initial(map_to_odom_broadcaster);
  // Create grid map.
  map.setFrameId("map");
  map.setGeometry(Length(30,30), 1);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

 // Position origin_position;
  // Index origin_index(0);
  // getPositionFromIndex(origin_position, origin_index, map.getLength(), map.getPosition(), map.getResolution(), Size(1,1));
  // map.setPosition(origin_position);
        
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = 0;
      map.at("occupancy", *it) = 0;
  }

  loadMapFromYAML(map);
  // Work with grid map in a loop.
  ros::Rate rate(30.0);
  while (ros::ok()) {
    transform_broadcaster_initial(map_to_odom_broadcaster);
    // Add data to grid map.
    ros::Time time = ros::Time::now();
    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    // Wait for next cycle.
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

