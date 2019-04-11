#include <octagon.h>

Octagon::Octagon(){
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
	image_transport::Publisher blue_filtered_pub = it.advertise("/octagon_task/bottom/blue_filtered", 1);
	image_transport::Publisher bottom_thresholded_pub = it.advertise("/octagon_task/bottom/thresholded", 1);
	image_transport::Publisher bottom_marked_pub = it.advertise("/octagon_task/bottom/marked", 1);
	ros::Publisher bottom_coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/octagon_task/bottom_bin_coordinates", 1000);

	image_transport::Subscriber image_raw_sub = it.subscribe("/front_camera/image_raw", 1, &Octagon::imageFrontCallback, this);


}

void Octagon::frontCallback(vision_tasks::octagonFrontRangeConfig &config, double level)
{
	Octagon::front_clahe_clip_ = config.front_clahe_clip;
	Octagon::front_clahe_grid_size_ = config.front_clahe_grid_size;
	Octagon::front_clahe_bilateral_iter_ = config.front_clahe_bilateral_iter;
	Octagon::front_balanced_bilateral_iter_ = config.front_balanced_bilateral_iter;
	Octagon::front_denoise_h_ = config.front_denoise_h;
	Octagon::front_low_h_ = config.front_low_h;
	Octagon::front_high_h_ = config.front_high_h;
	Octagon::front_low_s_ = config.front_low_s;
	Octagon::front_high_s_ = config.front_high_s;
	Octagon::front_low_v_ = config.front_low_v;
	Octagon::front_high_v_ = config.front_high_v;
	Octagon::front_opening_mat_point_ = config.front_opening_mat_point;
	Octagon::front_opening_iter_ = config.front_opening_iter;
	Octagon::front_closing_mat_point_ = config.front_closing_mat_point;
	Octagon::front_closing_iter_ = config.front_closing_iter;
}

void Octagon::bottomCallback(vision_tasks::octagonBottomRangeConfig &config, double level)
{
    Octagon::bottom_clahe_clip_ = config.bottom_clahe_clip;
	Octagon::bottom_clahe_grid_size_ = config.bottom_clahe_grid_size;
	Octagon::bottom_clahe_bilateral_iter_ = config.bottom_clahe_bilateral_iter;
	Octagon::bottom_balanced_bilateral_iter_ = config.bottom_balanced_bilateral_iter;
	Octagon::bottom_denoise_h_ = config.bottom_denoise_h;
	Octagon::bottom_low_h_ = config.bottom_low_h;
	Octagon::bottom_high_h_ = config.bottom_high_h;
	Octagon::bottom_low_s_ = config.bottom_low_s;
	Octagon::bottom_high_s_ = config.bottom_high_s;
	Octagon::bottom_low_v_ = config.bottom_low_v;
	Octagon::bottom_high_v_ = config.bottom_high_v;
	Octagon::bottom_opening_mat_point_ = config.bottom_opening_mat_point;
	Octagon::bottom_opening_iter_ = config.bottom_opening_iter;
	Octagon::bottom_closing_mat_point_ = config.bottom_closing_mat_point;
	Octagon::bottom_closing_iter_ = config.bottom_closing_iter;
}

void Octagon::imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg)
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

void Octagon::imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg)
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

void Octagon::spinThreadBottom() {
	dynamic_reconfigure::Server<vision_tasks::octagonBottomRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::octagonBottomRangeConfig>::CallbackType f;
	f = boost::bind(&Octagon::bottomCallback, this, _1, _2);
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

	while (ros::ok())
	{
		if (task_done) {
			task_done = false;
			break;
		}

		if (!image_bottom.empty())
		{
			image_bottom.copyTo(image_marked);
			//blue_filtered = vision_commons::Filter::blue_filter(image_, bottom_clahe_clip_, bottom_clahe_grid_size_, bottom_clahe_bilateral_iter_, bottom_balanced_bilateral_iter_, bottom_denoise_h_);
			if (bottom_high_h_ > bottom_low_h_ && bottom_high_s_ > bottom_low_s_ && bottom_high_v_ > bottom_low_v_)
			{
				cv::cvtColor(image_bottom, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, bottom_low_h_, bottom_high_h_, bottom_low_s_, bottom_high_s_, bottom_low_v_, bottom_high_v_);
				image_thresholded = vision_commons::Morph::open(image_thresholded, 2 * bottom_opening_mat_point_ + 1, bottom_opening_mat_point_, bottom_opening_mat_point_, bottom_opening_iter_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * bottom_closing_mat_point_ + 1, bottom_closing_mat_point_, bottom_closing_mat_point_, bottom_closing_iter_);
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

void Octagon::spinThreadFront() {
	dynamic_reconfigure::Server<vision_tasks::octagonFrontRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::octagonFrontRangeConfig>::CallbackType f;
	f = boost::bind(&Octagon::frontCallback, this, _1, _2);
	server.setCallback(f);
	
	std::cout << "front task Handling function start" << std::endl;	

	image_transport::ImageTransport it(nh);
	image_transport::Publisher front_blue_filtered_pub = it.advertise("/octagon_task/front/blue_filtered", 1);
	image_transport::Publisher front_thresholded_pub = it.advertise("/octagon_task/front/thresholded", 1);
	image_transport::Publisher front_marked_pub = it.advertise("/octagon_task/front/marked", 1);
	ros::Publisher front_coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/octagon_task/front_bin_coordinates", 1000);

	image_transport::Subscriber image_raw_sub = it.subscribe("/bottom_camera/image_raw", 1, &Octagon::imageFrontCallback, this);

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

		std::cout << "front Task Handling function over" << std::endl;	
	

	while (ros::ok())
	{
		if (task_done) {
			task_done = false;
			break;
		}

		if (!image_front.empty())
		{
			image_front.copyTo(image_marked);
			//blue_filtered = vision_commons::Filter::blue_filter(image_, clahe_clip_, clahe_grid_size_, clahe_bilateral_iter_, balanced_bilateral_iter_, denoise_h_);
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
				}
			}
			// front_blue_filtered_pub.publish(cv_bridge::CvImage(bin_center_message.header, "bgr8", blue_filtered).toImageMsg());
			front_thresholded_pub.publish(cv_bridge::CvImage(bin_center_message.header, "mono8", image_thresholded).toImageMsg());
			front_coordinates_pub.publish(bin_center_message);
			ROS_INFO("Bin Center Location (x, y, z) = (%.2f, %.2f, %.2f)", bin_center_message.point.x, bin_center_message.point.y, bin_center_message.point.z);
			front_marked_pub.publish(cv_bridge::CvImage(bin_center_message.header, "bgr8", image_marked).toImageMsg());
		}
		else
			ROS_INFO("Image empty");
		ros::spinOnce();
	}	
}


