#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

geometry_msgs::TwistStamped dvl_data;

void DVLCallback (const geometry_msgs::TwistWithCovarianceStamped msg) {
    dvl_data.header = msg.header;
    dvl_data.twist = msg.twist.twist;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "gazebo_dvl_driver");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>("/anahita/dvl_twist", 1000, &DVLCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/anahita/dvl/data", 1000);

    ros::Rate loop_rate(20);

    while (ros::ok()) {

        pub.publish(dvl_data);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
} 