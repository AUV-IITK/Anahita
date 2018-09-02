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
#include <std_msgs/Bool.h>
#include <vision_tasks/gateBottomRangeConfig.h>
#include <vision_commons/morph.h>
#include <vision_commons/contour.h>
#include <vision_commons/threshold.h>
#include <vision_commons/filter.h>

double clahe_clip_ = 4.0;
int clahe_grid_size_ = 8;
int clahe_bilateral_iter_ = 8;
int balanced_bilateral_iter_ = 4;
double denoise_h_ = 10.0;
int low_h_ = 10;
int low_s_ = 0;
int low_v_ = 0;
int high_h_ = 90;
int high_s_ = 255;
int high_v_ = 255;
int closing_mat_point_ = 1;
int closing_iter_ = 1;

cv::Mat image_;

std::string camera_frame_ = "auv-iitk";

void callback(vision_tasks::gateBottomRangeConfig &config, double level)
{
  clahe_clip_ = config.clahe_clip;
  clahe_grid_size_ = config.clahe_grid_size;
  clahe_bilateral_iter_ = config.clahe_bilateral_iter;
  balanced_bilateral_iter_ = config.balanced_bilateral_iter;
  denoise_h_ = config.denoise_h;
  low_h_ = config.low_h;
  high_h_ = config.high_h;
  low_s_ = config.low_s;
  high_s_ = config.high_s;
  low_v_ = config.low_v;
  high_v_ = config.high_v;
  closing_mat_point_ = config.closing_mat_point;
  closing_iter_ = config.closing_iter;
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
  ros::init(argc, argv, "gate_task_bottom");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<vision_tasks::gateBottomRangeConfig> server;
  dynamic_reconfigure::Server<vision_tasks::gateBottomRangeConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher blue_filtered_pub = it.advertise("/gate_task/bottom/blue_filtered", 1);
  image_transport::Publisher thresholded_pub = it.advertise("/gate_task/bottom/thresholded", 1);
  image_transport::Publisher marked_pub = it.advertise("/gate_task/bottom/marked", 1);
  ros::Publisher coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/gate_task/bottom/pipe_coordinates", 1000);
  ros::Publisher task_done_pub = nh.advertise<std_msgs::Bool>("/gate_task/done", 1000);

  image_transport::Subscriber bottom_image_sub = it.subscribe("/bottom_camera/image_raw", 1, imageCallback);

  cv::Scalar pipe_center_color(255, 255, 255);
  cv::Scalar image_center_color(0, 0, 0);
  cv::Scalar bounding_rectangle_color(255, 0, 0);

  cv::Mat blue_filtered;
  cv::Mat image_hsv;
  cv::Mat image_thresholded;
  geometry_msgs::PointStamped pipe_point_message;
  pipe_point_message.header.frame_id = camera_frame_.c_str();
  std::vector<std::vector<cv::Point> > contours;
  cv::RotatedRect bounding_rectangle;
  std_msgs::Bool task_done_message;
  cv::Mat image_marked;
  cv::Point2f vertices2f[4];
  cv::Point vertices[4];

  while (ros::ok())
  {
    if (!image_.empty())
    {
      image_.copyTo(image_marked);
      blue_filtered = vision_commons::Filter::blue_filter(image_, clahe_clip_, clahe_grid_size_, clahe_bilateral_iter_, balanced_bilateral_iter_, denoise_h_);
      if (high_h_ > low_h_ && high_s_ > low_s_ && high_v_ > low_v_)
      {
        cv::cvtColor(blue_filtered, image_hsv, CV_BGR2HSV);
        image_thresholded = vision_commons::Threshold::threshold(image_hsv, low_h_, high_h_, low_s_, high_s_, low_v_, high_v_);
        image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * closing_mat_point_ + 1, closing_mat_point_, closing_mat_point_, closing_iter_);
        contours = vision_commons::Contour::getBestX(image_thresholded, 1);
        if (contours.size() != 0)
        {
          bounding_rectangle = cv::minAreaRect(cv::Mat(contours[0]));
          pipe_point_message.header.stamp = ros::Time();
          pipe_point_message.point.x = (image_.size().height) / 2 - bounding_rectangle.center.y;
          pipe_point_message.point.y = bounding_rectangle.center.x - (image_.size().width) / 2;
          pipe_point_message.point.z = 0.0;
          task_done_message.data = pipe_point_message.point.x < 0;
          bounding_rectangle.points(vertices2f);
          for (int i = 0; i < 4; ++i)
          {
            vertices[i] = vertices2f[i];
          }
          cv::circle(image_marked, bounding_rectangle.center, 1, pipe_center_color, 8, 0);
          cv::circle(image_marked, cv::Point(image_.size().width / 2, image_.size().height / 2), 1, image_center_color, 8, 0);
          cv::fillConvexPoly(image_marked, vertices, 4, bounding_rectangle_color);
        }
      }
      blue_filtered_pub.publish(cv_bridge::CvImage(pipe_point_message.header, "bgr8", blue_filtered).toImageMsg());
      thresholded_pub.publish(cv_bridge::CvImage(pipe_point_message.header, "mono8", image_thresholded).toImageMsg());
      coordinates_pub.publish(pipe_point_message);
      ROS_INFO("Pipe Center (x, y) = (%.2f, %.2f)", pipe_point_message.point.x, pipe_point_message.point.y);
      task_done_pub.publish(task_done_message);
      ROS_INFO("Task done (bool) = %s", task_done_message.data ? "true" : "false");
      marked_pub.publish(cv_bridge::CvImage(pipe_point_message.header, "bgr8", image_marked).toImageMsg());
    }
    else
      ROS_INFO("Image empty");
    ros::spinOnce();
  }
  return 0;
}
