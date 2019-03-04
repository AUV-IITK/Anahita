#include <ros/ros.h>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <color_calibration/visionConfig.h>
#include <color_calibration/Dump.h>

int r_min, r_max, g_min, g_max, b_min, b_max;
int opening_mat_point, opening_iter, closing_mat_point, closing_iter;
int bilateral_iter;

bool save_params = false;

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

    if (config.save_params) {
        save_params = true;
        config.save_params = false;
    }

    ROS_INFO("A color calibration configuration request");
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "color_calibrate");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<color_calibration::visionConfig> server;
    dynamic_reconfigure::Server<color_calibration::visionConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::ServiceClient client = nh.serviceClient<color_calibration::Dump>("dump_parameters");
    color_calibration::Dump dump_srv;

    ros::Rate loop_rate(50);

    while (ros::ok()) {

        if (save_params) {
            nh.getParam("color_calibrate_object", dump_srv.request.filename);
            if (client.call(dump_srv))
            {
                ROS_INFO("Parameters dumped");
            }
            save_params = false;
        }
        
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}