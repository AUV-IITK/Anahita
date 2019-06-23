#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
geometry_msgs::Point mean_coordinate;

double z = 0;
int z_count = 0;
std::vector<double> z_coord;

double avg (std::vector<double> array) {
    double sum = 0;
    for (int i = 0; i < array.size(); i++) {
        sum += array[i];
    }
    double size = array.size();
    double avg_ = sum/size;

    return avg_;
}

bool inRange (double x, double avg, double thres) {
    if (avg + thres >= x || avg - thres <= x) {
        return true;
    }
    return false;
}

void zCallback (double msg) {
    z = msg;
    if (z_count < 10) { 
        z_coord[z_count] = z;
        mean_coordinate.z = avg (z_coord);
        z_count++;
    }
    else {
        mean_coordinate.z = avg (z_coord);
        if (inRange(z, mean_coordinate.z, 15)) {
            std::rotate (z_coord.begin(), z_coord.begin() + 1, z_coord.end());
            z_coord[9] = z;
        }
    }
}

void callback (const PointCloud::ConstPtr& msg) {
    double z_sum = 0;
    double x_sum = 0;
    double y_sum = 0;

    int size = msg->points.size();
    for (int i = 0; i < size; i++) {
        z_sum = z_sum + msg->points[i].z;
        x_sum = x_sum + msg->points[i].x;
        y_sum = y_sum + msg->points[i].y;
    }
    mean_coordinate.x = x_sum / size;
    mean_coordinate.y = y_sum / size;
    mean_coordinate.z = z_sum / size;

    mean_coordinate.z = mean_coordinate.z/1.75;
    zCallback (mean_coordinate.z);
}

int main (int argc, char** argv) {

    ros::init(argc, argv, "depth_calc");
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub = nh.subscribe<PointCloud>("/elas/point_cloud", 30, &callback);
    ros::Publisher mean_coord_pub = nh.advertise<geometry_msgs::Point>("/anahita/mean_coord", 1);

    ros::Rate loop_rate (20);

    mean_coordinate.x = 0;
    mean_coordinate.y = 0;
    mean_coordinate.z = 0;

    while (ros::ok()) {
        mean_coord_pub.publish(mean_coordinate);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}