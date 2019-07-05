#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

typedef geometry_msgs::PoseWithCovarianceStamped pose;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    ros::Rate rate(15);
    int count=0;
    while(ros::ok())
    {
        ros::Publisher pub= nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("anahita/landmarks",1000);
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
        ros::spin();
        rate.sleep();
        count++;
    }
    return 0;
}