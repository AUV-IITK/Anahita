#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

cv::Mat image;

void imageCB (const sensor_msgs::Image::ConstPtr &msg) {

    ROS_INFO("Getting Callbacks");

    try {
        image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch (cv::Exception &e)
    {
        ROS_ERROR("cv exception: %s", e.what());
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "create_image");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/varun/sensors/front_camera/image_raw", 1000, &imageCB);

    ros::Time::init();

    ros::Rate loop_rate(1);

    double then = ros::Time::now().toSec();

    int count = 0;

    std::string image_name = "";

    while (ros::ok()) {
        double now = ros::Time::now().toSec();
        double diff = now - then;

        image_name = "gate_" + to_string(count) + ".jpg";

        cv::imwrite(image_name, image);

        if (diff > 10) {
            break;
        }

        count++;
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}