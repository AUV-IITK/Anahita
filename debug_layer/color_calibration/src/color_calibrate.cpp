#include <ros/ros.h>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <color_calibration/visionConfig.h>

int r_min, r_max, g_min, g_max, b_min, b_max;
int opening_mat_point, opening_iter, closing_mat_point, closing_iter;
int bilateral_iter;

void callback (color_calibration::visionConfig &config, uint32_t level) {
    r_min = config.r_min;
    r_max = config.r_max;
    g_min = config.g_min;
    g_max = config.g_max;
    b_min = config.b_min;
    b_max = config.b_max;

    opening_mat_point = config.opening_mat_point;
    opening_iter = config.opening_iter;
    closing_mat_point = config.closing_mat_point;
    closing_iter = config.closing_iter;

    bilateral_iter = config.bilateral_iter;

    ROS_INFO("A color calibration configuration request");
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "color_calibrate");

    dynamic_reconfigure::Server<color_calibration::visionConfig> server;
    dynamic_reconfigure::Server<color_calibration::visionConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}