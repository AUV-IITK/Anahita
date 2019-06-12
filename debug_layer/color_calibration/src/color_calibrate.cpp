#include <ros/ros.h>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <color_calibration/visionConfig.h>
#include <color_calibration/Dump.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <filter.h>
#include <contour.h>
#include <morph.h>
#include <threshold.h>
#include <geometry.h>

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

cv::Mat image;

void callback (const sensor_msgs::Image::ConstPtr &msg) {
	try
	{
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

int main (int argc, char** argv) {
    ros::init (argc, argv, "color_calibrate");
    ros::NodeHandle nh;

    ros::Time::init();

    dynamic_reconfigure::Server<color_calibration::visionConfig> server;
    dynamic_reconfigure::Server<color_calibration::visionConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    image_transport::ImageTransport it(nh);

    bool convertToHSV = false;
    std::string camera = "left";
    nh.getParam("camera", camera);
    nh.getParam("convertToHSV", convertToHSV);
    image_transport::Subscriber image_sub = it.subscribe("/anahita/" + camera + "/image_raw", 1, &callback);
    image_transport::Publisher image_pub = it.advertise("/color_calibration/thresholded", 1);

    ros::ServiceClient client = nh.serviceClient<color_calibration::Dump>("dump_parameters");
    color_calibration::Dump dump_srv;

    ros::Rate loop_rate(20);

    cv::Mat image_thresholded;
    cv::Mat temp_src;
    sensor_msgs::ImagePtr msg;

    while (ros::ok()) {

        if (save_params) {
            nh.getParam("color_calibrate_object", dump_srv.request.filename);
            if (client.call(dump_srv))
            {
                ROS_INFO("Parameters dumped");
            }
            save_params = false;
        }
        if (!image.empty()) {
            temp_src = image.clone();
            if(convertToHSV)
            {
                cv::cvtColor(temp_src, temp_src, CV_BGR2HSV);
                ROS_INFO("Being converted to HSV format");
            }
            vision_commons::Filter::bilateral(temp_src, bilateral_iter);
            image_thresholded = vision_commons::Threshold::threshold(temp_src, b_min, b_max,
                                                                    g_min, g_max, r_min, r_max);
            vision_commons::Morph::open(image_thresholded, 2 * opening_mat_point + 1, 
                                        opening_mat_point, opening_mat_point, opening_iter);
            vision_commons::Morph::close(image_thresholded, 2 * closing_mat_point + 1, 
                                        closing_mat_point, closing_mat_point, closing_iter);
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg();
            image_pub.publish(msg);
        }
        else {
            ROS_INFO("Image empty");
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
