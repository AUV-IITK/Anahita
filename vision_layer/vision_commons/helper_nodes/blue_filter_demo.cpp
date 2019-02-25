#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>

#include <vision_commons/blueFilterDemoRangeConfig.h>
#include <filter.h>
#include <contour.h>
#include <morph.h>
#include <threshold.h>

double clahe_clip_ = 4.0;
int clahe_grid_size_ = 8;
int clahe_bilateral_iter_ = 8;
int balanced_bilateral_iter_ = 4;
double denoise_h_ = 10.0;

cv::Mat image_;

std::string camera_frame_ = "auv-iitk";

void callback(vision_commons::blueFilterDemoRangeConfig &config, double level)
{
  clahe_clip_ = config.clahe_clip;
  clahe_grid_size_ = config.clahe_grid_size;
  clahe_bilateral_iter_ = config.clahe_bilateral_iter;
  balanced_bilateral_iter_ = config.balanced_bilateral_iter;
  denoise_h_ = config.denoise_h;
}

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  try
  {
    image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "blue_filter_demo");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<vision_commons::blueFilterDemoRangeConfig> server;
  dynamic_reconfigure::Server<vision_commons::blueFilterDemoRangeConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher clahe_pub = it.advertise("/blue_filter_demo/clahe", 1);
  image_transport::Publisher white_balanced_pub = it.advertise("/blue_filter_demo/white_balanced", 1);
  image_transport::Publisher blue_filtered_pub = it.advertise("/blue_filter_demo/blue_filtered", 1);

  image_transport::Subscriber image_raw_sub = it.subscribe("/camera/image_raw", 1, imageCallback);

  cv::Mat clahe;
  cv::Mat white_balanced;
  cv::Mat blue_filtered;

  while (ros::ok())
  {
    if (!image_.empty())
    {
      ROS_INFO("Filtering...");
      clahe = vision_commons::Filter::clahe(image_, clahe_clip_, clahe_grid_size_);
      clahe.copyTo(white_balanced);
      white_balanced = vision_commons::Filter::balance_white(white_balanced);
      blue_filtered = vision_commons::Filter::blue_filter(image_, clahe_clip_, clahe_grid_size_, clahe_bilateral_iter_, balanced_bilateral_iter_, denoise_h_);
      clahe_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", clahe).toImageMsg());
      white_balanced_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", white_balanced).toImageMsg());
      blue_filtered_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", blue_filtered).toImageMsg());
    }
    else
      ROS_INFO("Image empty");
    ros::spinOnce();
  }
  return 0;
}
