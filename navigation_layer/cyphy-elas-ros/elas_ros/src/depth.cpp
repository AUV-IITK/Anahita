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

boost::shared_ptr<const PointCloud> cloud(new PointCloud);

geometry_msgs::Point mean_coordinate;

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

    cloud = msg;
}

int main (int argc, char** argv) {

    ros::init(argc, argv, "depth_calc");
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub = nh.subscribe<PointCloud>("/elas/point_cloud", 1, &callback);
    ros::Publisher mean_coord_pub = nh.advertise<geometry_msgs::Point>("/anahita/mean_coord", 1);
    ros::Publisher plane_coeff_pub = nh.advertise<std_msgs::Float32MultiArray>("/anahita/plane_coeff", 1);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    ros::Rate loop_rate (20);

    mean_coordinate.x = 0;
    mean_coordinate.y = 0;
    mean_coordinate.z = 0;

    std_msgs::Float32MultiArray normal_coeff;

    while (ros::ok()) {
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        normal_coeff.data.push_back(coefficients->values[0]);
        normal_coeff.data.push_back(coefficients->values[1]);
        normal_coeff.data.push_back(coefficients->values[2]);
        normal_coeff.data.push_back(coefficients->values[3]);

        mean_coord_pub.publish(mean_coordinate);
        plane_coeff_pub.publish(normal_coeff);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}