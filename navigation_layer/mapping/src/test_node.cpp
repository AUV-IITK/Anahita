#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

typedef geometry_msgs::PoseWithCovarianceStamped pose;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Rate rate(15);
    ros::Publisher pub= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/test_node/landmarks",1000);
     
    int count=0;
    while(ros::ok())
    {
        pose msg;
        std::string id("test %d", count);
        msg.header.frame_id=id;
        msg.pose.pose.position.x=1.0;
        msg.pose.pose.position.y=1.0;
        msg.pose.pose.position.z=1.0;
        msg.pose.pose.orientation.x=0.0;
        msg.pose.pose.orientation.y=0.0;
        msg.pose.pose.orientation.z=0.0;
        msg.pose.pose.orientation.w=1.0;
        pub.publish(msg);
        ROS_INFO("published message {%d}" ,count);
        ros::spinOnce();
        rate.sleep();
        count++;
    }
    return 0;
}