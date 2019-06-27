#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <master_layer/ChangeOdom.h>
#include <Eigen/Dense>

double z, y, x;
double z_avg, y_avg, x_avg;
double y_ml, z_ml, x_ml;

nav_msgs::Odometry odom_data;

std::vector<double> z_coord;
std::vector<double> y_coord;
std::vector<double> x_coord;

int z_count = 0;
int y_count = 0;
int x_count = 0;

bool odom_init = false;

Eigen::Matrix3f quaternion_matrix;

std::string odom_source = "dvl";

// I have not yet added anything to retrieve the current task
std::string current_task = "vetalas";

void ml_callback(const darknet_ros_msgs::BoundingBoxes msg)
{
    ROS_INFO("ML CALLBACK------------------------------------------------------------------------------ %f", sizeof(msg.bounding_boxes));
    double x_len, y_len;
    //int number_of_objects = sizeof(msg.bounding_boxes)/sizeof(msg.bounding_boxes[0]);
    //std::cout<<number_of_objects<<std::endl;
   
    for(int i = 0; i<1; i++)
    double x_len, y_len;
    int number_of_objects = sizeof(msg.bounding_boxes)/sizeof(msg.bounding_boxes[0]);
    for(int i = 0; i < number_of_objects; i++)
    {
	ROS_INFO("Current class: %s", msg.bounding_boxes[i].Class);
        if(msg.bounding_boxes[i].Class == current_task)
        {
            ROS_INFO("Inside ML Callback");
            y_ml = (msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax)/2 - 320; 
            z_ml = (msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax)/2 - 320;
            z_ml = 320 - (msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax)/2;
            y_len = (msg.bounding_boxes[i].ymax - msg.bounding_boxes[i].ymin);
            x_len = (msg.bounding_boxes[i].ymax - msg.bounding_boxes[i].ymin);
            x_ml = pow(y_len*y_len + x_len*x_len, 0.5);

        }
    }
}

bool changeOdom (master_layer::ChangeOdom::Request &req,
                master_layer::ChangeOdom::Response &res) {
    odom_source = req.odom;
    res.success = true;
    return true;
}

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

void zCallback (const std_msgs::Float32 msg) {
    z = -msg.data;
    if (z_count < 10) { 
        z_coord[z_count] = z;
        z_avg = avg (z_coord);
        z_count++;
    }
    else {
        z_avg = avg (z_coord);
        if (inRange(z, z_avg, 15)) {
            std::rotate (z_coord.begin(), z_coord.begin() + 1, z_coord.end());
            z_coord[9] = z;
        }
    }
}

void yCallback (const std_msgs::Float32 msg) {
    y = msg.data;
    if (y_count < 10) { 
        y_coord[y_count] = y;
        y_avg = avg (y_coord);
        y_count++;
    }
    else {
        y_avg = avg (y_coord);
        if (inRange(y, y_avg, 15)) {
            std::rotate (y_coord.begin(), y_coord.begin() + 1, y_coord.end());
            y_coord[9] = y;
        }
    }
}

void xCallback (const geometry_msgs::Point msg) {
    x = msg.z;
    if (x_count < 10) { 
        x_coord[x_count] = x;
        x_avg = avg (x_coord);
        x_count++;
    }
    else {
        x_avg = avg (x_coord);
        if (inRange(x, x_avg, 1)) {
            std::rotate (x_coord.begin(), x_coord.begin() + 1, x_coord.end());
            x_coord[9] = x;
        }
    }
}

void dvlCallback (const nav_msgs::Odometry msg) {
    odom_data.pose.pose.position.x = msg.pose.pose.position.x;
    odom_data.pose.pose.position.y = msg.pose.pose.position.y;
    odom_data.pose.pose.position.z = msg.pose.pose.position.z;
    odom_data.pose.pose.orientation.x = msg.pose.pose.orientation.x;
    odom_data.pose.pose.orientation.y = msg.pose.pose.orientation.y;
    odom_data.pose.pose.orientation.z = msg.pose.pose.orientation.z;
    odom_data.pose.pose.orientation.w = msg.pose.pose.orientation.w;
    odom_data.twist.twist.linear.x = msg.twist.twist.linear.x;
    odom_data.twist.twist.linear.y = msg.twist.twist.linear.y;
    odom_data.twist.twist.linear.z = msg.twist.twist.linear.z;
    odom_data.twist.twist.angular.x = msg.twist.twist.angular.x; 
    odom_data.twist.twist.angular.y = msg.twist.twist.angular.y;
    odom_data.twist.twist.angular.z = msg.twist.twist.angular.z;
    odom_data.header.frame_id = msg.header.frame_id;
    odom_data.child_frame_id = msg.child_frame_id;
    odom_data.header.stamp = odom_data.header.stamp;
    odom_init = true;
}

void qmatrixUpdate () {
    geometry_msgs::Quaternion q = odom_data.pose.pose.orientation;
    quaternion_matrix(0, 0) = 1 - 2*q.y*q.y - 2*q.z*q.z;
    quaternion_matrix(0, 1) = 2*q.x*q.y - 2*q.z*q.w;
    quaternion_matrix(0, 2) = 2*q.x*q.z + 2*q.y*q.w;
    quaternion_matrix(1, 0) = 2*q.x*q.y + 2*q.z*q.w;
    quaternion_matrix(1, 1) = 1 - 2*q.x*q.x - 2*q.z*q.z;
    quaternion_matrix(1, 2) = 2*q.y*q.z - 2*q.x*q.w;
    quaternion_matrix(2, 0) = 2*q.x*q.z - 2*q.y*q.w;
    quaternion_matrix(2, 1) = 2*q.y*q.z + 2*q.x*q.w;
    quaternion_matrix(2, 2) = 1 - 2*q.x*q.x - 2*q.y*q.y;
}

void transform (geometry_msgs::Point& p) {
    Eigen::Vector3f p_world;
    p_world(0) = p.x;
    p_world(1) = p.y;
    p_world(2) = p.z;
    qmatrixUpdate ();
    Eigen::Vector3f p_body = quaternion_matrix.transpose()*p_world;
    p.x = p_body(0);
    p.y = p_body(1);
    p.z = p_body(2);
}

int main (int argc, char** argv) {

    ros::init (argc, argv, "local_vision_odom");
    ros::NodeHandle nh;

    ros::Subscriber z_sub = nh.subscribe("/anahita/front_camera/z_coordinate", 100, &zCallback);
    ros::Subscriber y_sub = nh.subscribe("/anahita/front_camera/y_coordinate", 100, &yCallback);
    ros::Subscriber odom_sub = nh.subscribe("/anahita/pose_gt/relay", 100, &dvlCallback);
    ros::Subscriber mean_coord_sub = nh.subscribe("/anahita/mean_coord", 100, &xCallback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/anahita/pose_gt", 100);
    ros::Subscriber ml_sub = nh.subscribe("/anahita/bounding_boxes", 1, &ml_callback);

    ros::ServiceServer service = nh.advertiseService("odom_source", changeOdom);
    ros::Rate loop_rate(20);

    nav_msgs::Odometry odom_msg;

    z_coord.resize(10);
    y_coord.resize(10);
    x_coord.resize(10);

    while (ros::ok()) {

        if (odom_source == "dvl") {
            odom_msg = odom_data;
        }
        else if (odom_source == "vision") {
            odom_msg = odom_data;
            odom_msg.pose.pose.position.y = y_avg/1000.0;
            odom_msg.pose.pose.position.z = z_avg/1000.0;
            if (odom_init) transform (odom_msg.pose.pose.position);
  	    ROS_INFO("VALUES---------- %f %f %f", -x_avg/1000, y_avg/200, -z_avg/1000);
        }
        else if (odom_source == "vision_ml"){
            odom_msg = odom_data;
            //odom_msg.pose.pose.position.x = 0;
            odom_msg.pose.pose.position.y = y_ml/200.0;
            //odom_msg.pose.pose.position.z = 0;
//            transform (odom_msg.pose.pose.position);
	    ROS_INFO("VALUES---------------------------------------------------------------------- %f %f %f", -x_ml/1000, y_ml/200, -z_ml/1000);
            odom_msg.pose.pose.position.x = -x_ml;
            odom_msg.pose.pose.position.y = y_ml;
            odom_msg.pose.pose.position.z = z_ml;
        }
        else if (odom_source == "stereo_vision"){
            odom_msg = odom_data;
            odom_msg.pose.pose.position.y = y_avg/200.0;
            odom_msg.pose.pose.position.z = z_avg/200.0;
            if (odom_init) transform (odom_msg.pose.pose.position);
            odom_msg.pose.pose.position.x = -x_avg;
        }
        else {
            ROS_INFO("Invalid odom source");
            return 1;
        }

        odom_pub.publish(odom_msg);
        
        loop_rate.sleep();
        ros::spinOnce();
    }

}
