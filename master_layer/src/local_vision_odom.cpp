#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

double x, y, z;
geometry_msgs::Quaternion quat;

void xCallback (const std_msgs::Float32 msg) {
    x = -msg.data/100;
}

void yCallback (const std_msgs::Float32 msg) {
    y = -msg.data/100;
}

void zCallback (const std_msgs::Float32 msg) {
    z = -msg.data/100;
}

void imuCallback (const sensor_msgs::Imu msg) {
    quat.x = msg.orientation.x;
    quat.y = msg.orientation.y;
    quat.z = msg.orientation.z;
    quat.w = msg.orientation.w;
}

int main (int argc, char** argv) {

    ros::init (argc, argv, "local_vision_odom");
    ros::NodeHandle nh;

    ros::Subscriber x_sub = nh.subscribe("/anahita/x_coordinate", 100, &xCallback);
    ros::Subscriber y_sub = nh.subscribe("/anahita/y_coordinate", 100, &yCallback);
    ros::Subscriber z_sub = nh.subscribe("/anahita/z_coordinate", 100, &zCallback);
    ros::Subscriber imu_sub = nh.subscribe("/anahita/imu", 100, &imuCallback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/anahita/pose_gt", 100);

    ros::Rate loop_rate(20);

    nav_msgs::Odometry odom_msg;

    while (ros::ok()) {
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;
        odom_msg.pose.pose.orientation.x = quat.x;
        odom_msg.pose.pose.orientation.y = quat.y;
        odom_msg.pose.pose.orientation.z = quat.z;
        odom_msg.pose.pose.orientation.w = quat.w;

        odom_pub.publish(odom_msg);
        
        loop_rate.sleep();
        ros::spinOnce();
    }

}