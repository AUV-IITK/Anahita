#include <buoy.h>

Buoy::Buoy(){
	this->clahe_clip_ = 0.15;
	this->clahe_grid_size_ = 3;
	this->clahe_bilateral_iter_ = 2;
	this->balanced_bilateral_iter_ = 4;
	this->denoise_h_ = 10.0;
	this->front_low_h_ = 0;
	this->front_high_h_ = 25;
	this->front_low_s_ = 245;
	this->front_high_s_ = 255;
	this->front_low_v_ = 78;
	this->front_high_v_ = 115;
	this->front_opening_mat_point_ = 1;
	this->front_opening_iter_ = 3;
	this->front_closing_mat_point_ = 1;
	this->front_closing_iter_ = 0;
	this->camera_frame_ = "auv-iitk";
	this->current_color = 0;
	image_transport::ImageTransport it(nh);
	this->blue_filtered_pub = it.advertise("/buoy_task/blue_filtered", 1);
	this->image_raw_sub = it.subscribe("/front_camera/image_raw", 1, &Buoy::imageCallback, this);
}

void Buoy::switchColor(int color)
{
	if(color > 2)
		std::cerr << "Changing to wrong buoy color, use 0-2 for the the different colors" << std::endl;
	else
	{
		current_color = color;		
		this->front_low_h_ = this->data_low_h[current_color];
		this->front_high_h_ = this->data_high_h[current_color];
		this->front_low_s_ = this->data_low_s[current_color];
		this->front_high_s_ = this->data_high_s[current_color];
		this->front_low_v_ = this->data_low_v[current_color];
		this->front_high_v_ = this->data_high_v[current_color];
	}
	std::cout << "Colour changed successfully" << std::endl;
}

void Buoy::callback(vision_tasks::buoyRangeConfig &config, double level)
{
	if(Buoy::current_color != config.color)
	{
		config.color = current_color;
		config.low_h = this->data_low_h[current_color];
		config.high_h = this->data_high_h[current_color];
		config.low_s = this->data_low_s[current_color];
		config.high_s = this->data_high_s[current_color];
		config.low_v = this->data_low_v[current_color];
		config.high_v = this->data_high_v[current_color];
	}
	this->clahe_clip_ = config.clahe_clip;
	this->clahe_grid_size_ = config.clahe_grid_size;
	this->clahe_bilateral_iter_ = config.clahe_bilateral_iter;
	this->balanced_bilateral_iter_ = config.balanced_bilateral_iter;
	this->denoise_h_ = config.denoise_h;
	this->front_low_h_ = config.low_h;
	this->front_high_h_ = config.high_h;
	this->front_low_s_ = config.low_s;
	this->front_high_s_ = config.high_s;
	this->front_low_v_ = config.low_v;
	this->front_high_v_ = config.high_v;
	this->front_opening_mat_point_ = config.opening_mat_point;
	this->front_opening_iter_ = config.opening_iter;
	this->front_closing_mat_point_ = config.closing_mat_point;
	this->front_closing_iter_ = config.closing_iter;
}

void Buoy::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	try
	{
		image_front= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
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

void Buoy::spinThreadFront(){

	dynamic_reconfigure::Server<vision_tasks::buoyRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::buoyRangeConfig>::CallbackType f;
	f = boost::bind(&Buoy::callback, this, _1, _2);
	server.setCallback(f);

	cv::Scalar buoy_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar enclosing_circle_color(149, 255, 23);
	cv::Scalar contour_color(255, 0, 0);

	cv::Mat blue_filtered;
	cv::Mat image_hsv;
	cv::Mat image_thresholded;
	cv::Mat image_marked;
	std::vector<std::vector<cv::Point> > contours;
	cv::Rect bounding_rectangle;
	std::vector<cv::Point2f> center(1);
	std::vector<float> radius(1);
	geometry_msgs::PointStamped buoy_point_message;
	buoy_point_message.header.frame_id = camera_frame_.c_str();
	cv::RotatedRect min_ellipse;
	ros::Rate loop_rate(25);

	std_msgs::Bool detection_bool;

	while (ros::ok())
	{	
		if (close_task) {
			break;
		}
		if (!image_front.empty())
		{
			image_front.copyTo(image_marked);

			int64 t0_ = cv::getTickCount();
			int64 t1_ = cv::getTickCount();
			// ROS_INFO("Time taken by blue-filter: %lf", (t1_-t0_)/cv::getTickFrequency());

			//blue_filtered = image_;
			if (front_high_h_ > front_low_h_ && front_high_s_ > front_low_s_ && front_high_v_ > front_low_v_)
			{	

				int64 t0 = cv::getTickCount();
				std::cout<<"Colour being detected is: " << current_color << std::endl;
				cv::cvtColor(blue_filtered, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, front_low_h_, front_high_h_, front_low_s_, front_high_s_, front_low_v_, front_high_v_);
				int64 t1 = cv::getTickCount();
				// ROS_INFO("Time taken by thresholding: %lf", (t1-t0)/cv::getTickFrequency());

				image_thresholded = vision_commons::Morph::open(image_thresholded, 2 * front_opening_mat_point_ + 1, front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * front_closing_mat_point_ + 1, front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);

				int64 t2 = cv::getTickCount();
				contours = vision_commons::Contour::getBestX(image_thresholded, 2);
				int64 t3 = cv::getTickCount();
				// ROS_INFO("Time taken by contours: %lf", (t3-t2)/cv::getTickFrequency());

				if (contours.size() != 0)
				{
					int index = 0;
					bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
					cv::minEnclosingCircle(contours[index], center[0], radius[0]);
					buoy_point_message.header.stamp = ros::Time();
					front_x_coordinate.data = pow(radius[0] / 7526.5, -.92678);
 					front_y_coordinate.data = center[0].x - ((float)image_front.size().width) / 2;
					front_z_coordinate.data = ((float)image_front.size().height) / 2 - center[0].y;
					ROS_INFO("Buoy Location (x, y, z) = (%.2f, %.2f, %.2f)", front_x_coordinate.data, front_y_coordinate.data, front_z_coordinate.data);
					ROS_INFO("Maximum Contour area: %.2f", contourArea(contours[index]));
					if(contourArea(contours[index]) > 1000 && abs(front_y_coordinate.data) < 220 && abs(front_z_coordinate.data) < 300) 
						detection_bool.data = true;
					else
						detection_bool.data = false;						
					detection_pub.publish(detection_bool);

					cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x) / 2, (bounding_rectangle.br().y + bounding_rectangle.tl().y) / 2), 1, buoy_center_color, 8, 0);
					cv::circle(image_marked, cv::Point(image_front.size().width / 2, image_front.size().height / 2), 1, image_center_color, 8, 0);
					cv::circle(image_marked, center[0], (int)radius[0], enclosing_circle_color, 2, 8, 0);
					for (int i = 0; i < contours.size(); i++)
					{
						cv::drawContours(image_marked, contours, i, contour_color, 1);
					}
					int64 tend_ = cv::getTickCount();
					// ROS_INFO("Time taken by entire buoy node: %lf", (tend_-t0_)/cv::getTickFrequency());				
					
				}
				else
				{
					ROS_INFO("Object not being detected");
					detection_bool.data = false;
					detection_pub.publish(detection_bool);
				}	
			}
			// blue_filtered_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "bgr8", blue_filtered).toImageMsg());		
      front_thresholded_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "mono8", image_thresholded).toImageMsg());
			front_x_coordinates_pub.publish(front_x_coordinate);
			front_y_coordinates_pub.publish(front_y_coordinate);
			front_z_coordinates_pub.publish(front_z_coordinate);
			front_marked_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "bgr8", image_marked).toImageMsg());
      /* TO CHECK CODE DURING POOL TESTING
      x_coordinate.data = -x_coordinate.data/100;
			y_coordinate.data = -y_coordinate.data/100;
			z_coordinate.data = -z_coordinate.data/100;

			x_coordinates_pub.publish(x_coordinate);
			y_coordinates_pub.publish(y_coordinate);
			bool enable_pressure = false;
			nh.getParam("/enable_pressure", enable_pressure);
			if (!enable_pressure) {
				z_coordinates_pub.publish(z_coordinate);
			}
			marked_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "bgr8", image_marked).toImageMsg());
      */
		}
		else
		{
			ROS_INFO("Image empty");
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}
