#include <markerDropper.h>
#include"base_class.h"
MarkerDropper::MarkerDropper(){
	this->front_clahe_clip_ = 4.0;
    this->front_clahe_grid_size_ = 8;
    this->front_clahe_bilateral_iter_ = 8;
    this->front_balanced_bilateral_iter_ = 4;
    this->front_denoise_h_ = 10.0;
    this->front_low_h_ = 0;
    this->front_high_h_ = 20;
    this->front_low_s_ = 0;
    this->front_high_s_ = 92;
    this->front_low_v_ = 46;
    this->front_high_v_ = 255;
    this->front_closing_mat_point_ = 1;
    this->front_closing_iter_ = 1;
    this->front_canny_threshold_low_ = 0;
    this->front_canny_threshold_high_ = 1000;
    this->front_canny_kernel_size_ = 3;
    this->front_hough_threshold_ = 0;
    this->front_hough_minline_ = 0;
    this->front_hough_maxgap_ = 0;
    this->front_hough_angle_tolerance_ = 0.0;
    this->front_gate_distance_tolerance_ = 50.0;
    this->front_gate_angle_tolerance_ = 0.0;

    this->bottom_clahe_clip_ = 4.0;
    this->bottom_clahe_grid_size_ = 8;
    this->bottom_clahe_bilateral_iter_ = 8;
    this->bottom_balanced_bilateral_iter_ = 4;
    this->bottom_denoise_h_ = 10.0;
    this->bottom_low_h_ = 10;
    this->bottom_low_s_ = 0;
    this->bottom_low_v_ = 0;
    this->bottom_high_h_ = 90;
    this->bottom_high_s_ = 255;
    this->bottom_high_v_ = 255;
    this->bottom_closing_mat_point_ = 1;
    this->bottom_closing_iter_ = 1;

    this->camera_frame_ = "auv-iitk";

	image_transport::ImageTransport it(nh);
	
	this->bottom_blue_filtered_pub = it.advertise("/markerdropper_task/bottom/blue_filtered", 1);
	this->front_blue_filtered_pub = it.advertise("/markerdropper_task/front/blue_filtered", 1);
	this->bottom_image_raw_sub = it.subscribe("/anahita/bottom_camera/image_raw", 1, &MarkerDropper::imageBottomCallback, this);
	this->front_image_raw_sub = it.subscribe("/anahita/front_camera/image_raw", 1, &MarkerDropper::imageFrontCallback, this);
}


void MarkerDropper::frontCallback(vision_tasks::markerDropperFrontRangeConfig &config, double level)
{
	MarkerDropper::front_clahe_clip_ = config.front_clahe_clip;
	MarkerDropper::front_clahe_grid_size_ = config.front_clahe_grid_size;
	MarkerDropper::front_clahe_bilateral_iter_ = config.front_clahe_bilateral_iter;
	MarkerDropper::front_balanced_bilateral_iter_ = config.front_balanced_bilateral_iter;
	MarkerDropper::front_denoise_h_ = config.front_denoise_h;
	MarkerDropper::front_low_h_ = config.front_low_h;
	MarkerDropper::front_high_h_ = config.front_high_h;
	MarkerDropper::front_low_s_ = config.front_low_s;
	MarkerDropper::front_high_s_ = config.front_high_s;
	MarkerDropper::front_low_v_ = config.front_low_v;
	MarkerDropper::front_high_v_ = config.front_high_v;
	MarkerDropper::front_opening_mat_point_ = config.front_opening_mat_point;
	MarkerDropper::front_opening_iter_ = config.front_opening_iter;
	MarkerDropper::front_closing_mat_point_ = config.front_closing_mat_point;
	MarkerDropper::front_closing_iter_ = config.front_closing_iter;
};

void MarkerDropper::bottomCallback(vision_tasks::markerDropperBottomRangeConfig &config, double level)
{
	MarkerDropper::bottom_clahe_clip_ = config.bottom_clahe_clip;
	MarkerDropper::bottom_clahe_grid_size_ = config.bottom_clahe_grid_size;
	MarkerDropper::bottom_clahe_bilateral_iter_ = config.bottom_clahe_bilateral_iter;
	MarkerDropper::bottom_balanced_bilateral_iter_ = config.bottom_balanced_bilateral_iter;
	MarkerDropper::bottom_denoise_h_ = config.bottom_denoise_h;
	MarkerDropper::bottom_low_h_ = config.bottom_low_h;
	MarkerDropper::bottom_high_h_ = config.bottom_high_h;
	MarkerDropper::bottom_low_s_ = config.bottom_low_s;
	MarkerDropper::bottom_high_s_ = config.bottom_high_s;
	MarkerDropper::bottom_low_v_ = config.bottom_low_v;
	MarkerDropper::bottom_high_v_ = config.bottom_high_v;
	MarkerDropper::bottom_opening_mat_point_ = config.bottom_opening_mat_point;
	MarkerDropper::bottom_opening_iter_ = config.bottom_opening_iter;
	MarkerDropper::bottom_closing_mat_point_ = config.bottom_closing_mat_point;
	MarkerDropper::bottom_closing_iter_ = config.bottom_closing_iter;
};

void MarkerDropper::imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_img_ptr;
	try
	{
		image_front = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (cv::Exception &e)
	{
		ROS_ERROR("cv exception: %s", e.what());
	}
};

void MarkerDropper::imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_img_ptr;
	try
	{
		image_bottom = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (cv::Exception &e)
	{
		ROS_ERROR("cv exception: %s", e.what());
	}
};

void MarkerDropper::spinThreadBottom() {
	dynamic_reconfigure::Server<vision_tasks::markerDropperBottomRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::markerDropperBottomRangeConfig>::CallbackType f;
	f = boost::bind(&MarkerDropper::bottomCallback, this, _1, _2);
	server.setCallback(f);

	cv::Scalar bin_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar enclosing_circle_color(149, 255, 23);
	cv::Scalar contour_color(255, 0, 0);

	cv::Mat blue_filtered;
	cv::Mat image_hsv;
	cv::Mat image_thresholded;
	cv::Mat image_marked;
	std::vector<std::vector<cv::Point> > contours, polygons(3);
	std::vector<cv::Moments> mu(3);
	std::vector<cv::Point2f> mc(2);
	cv::Rect bounding_rectangle;
	std::vector<cv::Point2f> center(1);
	std::vector<float> radius(1);
	geometry_msgs::PointStamped bin_center_message;
	bin_center_message.header.frame_id = camera_frame_.c_str();
	cv::RotatedRect min_ellipse;
	std_msgs::Bool detection_bool;

	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_bottom.empty())
		{
			image_bottom.copyTo(image_marked);
			// blue_filtered = vision_commons::Filter::blue_filter(image_bottom, bottom_clahe_clip_, bottom_clahe_grid_size_, bottom_clahe_bilateral_iter_, bottom_balanced_bilateral_iter_, bottom_denoise_h_);
			if (bottom_high_h_ > bottom_low_h_ && bottom_high_s_ > bottom_low_s_ && bottom_high_v_ > bottom_low_v_)
			{
				cv::cvtColor(image_bottom, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, bottom_low_h_, bottom_high_h_, bottom_low_s_, bottom_high_s_, bottom_low_v_, bottom_high_v_);
				image_thresholded = vision_commons::Morph::open(image_thresholded, 2 * bottom_opening_mat_point_ + 1, bottom_opening_mat_point_, bottom_opening_mat_point_, bottom_opening_iter_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * bottom_closing_mat_point_ + 1, bottom_closing_mat_point_, bottom_closing_mat_point_, bottom_closing_iter_);
				contours = vision_commons::Contour::getBestX(image_thresholded, 2);
				if (contours.size() != 0)
				{
					int index = 0;
					for(int i = 0; i<contours.size(); i++)
					{
						approxPolyDP(cv::Mat(contours[i]), polygons[i], 3, true);
     					mu[i] = moments( polygons[i], false ); 
						mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );			
					}
					bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
					if (contours.size() >= 2)
					{
						cv::Rect bounding_rectangle2 = cv::boundingRect(cv::Mat(contours[1]));
						if ((bounding_rectangle2.br().y + bounding_rectangle2.tl().y) > (bounding_rectangle.br().y + bounding_rectangle.tl().y))
						{
							index = 1;
							bounding_rectangle = bounding_rectangle2;
						}
					}
					bin_center_message.header.stamp = ros::Time();
					bin_center_message.point.x = pow(radius[0] / 7526.5, -.92678);
					bin_center_message.point.y = mc[0].x - ((float)image_bottom.size().width) / 2;
					bin_center_message.point.z = ((float)image_bottom.size().height) / 2 - mc[0].y;
					ROS_INFO("Bin Center Location (x, y, z) = (%.2f, %.2f, %.2f)", bin_center_message.point.x, bin_center_message.point.y, bin_center_message.point.z);
					cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x) / 2, (bounding_rectangle.br().y + bounding_rectangle.tl().y) / 2), 1, bin_center_color, 8, 0);
					cv::circle(image_marked, cv::Point(image_bottom.size().width / 2, image_bottom.size().height / 2), 1, image_center_color, 8, 0);
					cv::circle(image_marked, center[0], (int)radius[0], enclosing_circle_color, 2, 8, 0);
					for (int i = 0; i < contours.size(); i++)
					{
						cv::drawContours(image_marked, polygons, i, contour_color, 1);
					}
				}
			}
			// bottom_blue_filtered_pub.publish(cv_bridge::CvImage(bin_center_message.header, "bgr8", blue_filtered).toImageMsg());
			bottom_thresholded_pub.publish(cv_bridge::CvImage(bin_center_message.header, "mono8", image_thresholded).toImageMsg());
			bottom_coordinates_pub.publish(bin_center_message);
			ROS_INFO("Bin Center Location (x, y, z) = (%.2f, %.2f, %.2f)", bin_center_message.point.x, bin_center_message.point.y, bin_center_message.point.z);
			bottom_marked_pub.publish(cv_bridge::CvImage(bin_center_message.header, "bgr8", image_marked).toImageMsg());
		}
		else
			ROS_INFO("Image empty");
		ros::spinOnce();
	}
}

void MarkerDropper::spinThreadFront () {
	dynamic_reconfigure::Server<vision_tasks::markerDropperFrontRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::markerDropperFrontRangeConfig>::CallbackType f;
	f = boost::bind(&MarkerDropper::frontCallback, this, _1, _2);
	server.setCallback(f);
	
	cv::Scalar bin_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar enclosing_circle_color(149, 255, 23);
	cv::Scalar contour_color(255, 0, 0);

	cv::Mat blue_filtered;
	cv::Mat image_hsv;
	cv::Mat image_thresholded;
	cv::Mat image_marked;
	std::vector<std::vector<cv::Point> > contours, polygons(3);
	std::vector<cv::Moments> mu(3);
	std::vector<cv::Point2f> mc(2);
	cv::Rect bounding_rectangle;
	std::vector<cv::Point2f> center(1);
	std::vector<float> radius(1);
	geometry_msgs::PointStamped bin_center_message;
	bin_center_message.header.frame_id = camera_frame_.c_str();
	cv::RotatedRect min_ellipse;
	std_msgs::Bool detection_bool;

	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_front.empty())
		{
			image_front.copyTo(image_marked);
			//blue_filtered = vision_commons::Filter::blue_filter(image_front, clahe_clip_, clahe_grid_size_, clahe_bilateral_iter_, balanced_bilateral_iter_, denoise_h_);
			if (front_high_h_ > front_low_h_ && front_high_s_ > front_low_s_ && front_high_v_ > front_low_v_)
			{
				cv::cvtColor(image_front, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, front_low_h_, front_high_h_, front_low_s_,front_high_s_, front_low_v_, front_high_v_);
				image_thresholded = vision_commons::Morph::open(image_thresholded, 2 * front_opening_mat_point_ + 1, front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * front_closing_mat_point_ + 1, front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);
				contours = vision_commons::Contour::getBestX(image_thresholded, 3);
				if (contours.size() != 0)
				{
					int index = 0;
					for(int i = 0; i<contours.size(); i++)
					{
						approxPolyDP(cv::Mat(contours[i]), polygons[i], 3, true);
     					mu[i] = moments( polygons[i], false ); 
						mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );			
					}
					bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
					if (contours.size() >= 2)
					{
						cv::Rect bounding_rectangle2 = cv::boundingRect(cv::Mat(contours[1]));
						if ((bounding_rectangle2.br().y + bounding_rectangle2.tl().y) > (bounding_rectangle.br().y + bounding_rectangle.tl().y))
						{
							index = 1;
							bounding_rectangle = bounding_rectangle2;
						}
					}
					bin_center_message.header.stamp = ros::Time();
					bin_center_message.point.x = pow(radius[0] / 7526.5, -.92678);
					bin_center_message.point.y = mc[0].x - ((float)image_front.size().width) / 2;
					bin_center_message.point.z = ((float)image_front.size().height) / 2 - mc[0].y;
					ROS_INFO("Bin Center Location (x, y, z) = (%.2f, %.2f, %.2f)", bin_center_message.point.x, bin_center_message.point.y, bin_center_message.point.z);
					cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x) / 2, (bounding_rectangle.br().y + bounding_rectangle.tl().y) / 2), 1, bin_center_color, 8, 0);
					cv::circle(image_marked, cv::Point(image_front.size().width / 2, image_front.size().height / 2), 1, image_center_color, 8, 0);
					cv::circle(image_marked, center[0], (int)radius[0], enclosing_circle_color, 2, 8, 0);
					for (int i = 0; i < contours.size(); i++)
					{
						cv::drawContours(image_marked, polygons, i, contour_color, 1);
					}
					detection_bool.data = true;
					
				}
			}
			// front_blue_filtered_pub.publish(cv_bridge::CvImage(bin_center_message.header, "bgr8", blue_filtered).toImageMsg());
			front_thresholded_pub.publish(cv_bridge::CvImage(bin_center_message.header, "mono8", image_thresholded).toImageMsg());
			front_coordinates_pub.publish(bin_center_message);
			ROS_INFO("Bin Center Location (x, y, z) = (%.2f, %.2f, %.2f)", bin_center_message.point.x, bin_center_message.point.y, bin_center_message.point.z);
			front_marked_pub.publish(cv_bridge::CvImage(bin_center_message.header, "bgr8", image_marked).toImageMsg());
			detection_pub.publish(detection_bool);
		}
		else
			ROS_INFO("Image empty");
		ros::spinOnce();
	}
}
