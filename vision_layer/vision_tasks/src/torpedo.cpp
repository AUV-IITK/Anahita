#include <torpedo.h>

Torpedo::Torpedo(){
	this->clahe_clip_ = 0.15;
	this->clahe_grid_size_ = 3;
	this->clahe_bilateral_iter_ = 2;
	this->balanced_bilateral_iter_ = 4;
	this->denoise_h_ = 5.6;
	this->front_low_h_ = 53;
	this->front_high_h_ = 86;
	this->front_low_s_ = 128;
	this->front_high_s_ = 255;
	this->front_low_v_ = 104;
	this->front_high_v_ = 202;
	this->front_opening_mat_point_ = 2;
	this->front_opening_iter_ = 1;
	this->front_closing_mat_point_ = 2;
	this->front_closing_iter_ = 3;
	this->camera_frame_ = "auv-iitk";
	image_transport::ImageTransport it(nh);
	this->front_x_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/x_coordinate", 1000);
	this->front_y_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/y_coordinate", 1000);
	this->front_z_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/z_coordinate", 1000);
	this->detection_pub = nh.advertise<std_msgs::Bool>("/detected", 1000);
	this->blue_filtered_pub = it.advertise("/torpedo_task/blue_filtered", 1);
	this->front_thresholded_pub = it.advertise("/torpedo_task/thresholded", 1);
	this->front_marked_pub = it.advertise("/torpedo_task/marked", 1);
	this->image_raw_sub = it.subscribe("/anahita/front_camera/image_raw", 1, &Torpedo::imageCallback, this);
}

void Torpedo::switchColor(int color)
{
	if(color > 1)
		std::cerr << "Changing to wrong torpedo color, use 0-1 for the the different colors" << std::endl;
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


void Torpedo::callback(vision_tasks::torpedoRangeConfig &config, double level)
{
	if(Torpedo::current_color != config.color)
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
};

void Torpedo::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	try
	{
		image_front = cv_bridge::toCvCopy(msg, "bgr8")->image;
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

void Torpedo::spinThreadFront(){

	dynamic_reconfigure::Server<vision_tasks::torpedoRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::torpedoRangeConfig>::CallbackType f;
	f = boost::bind(&Torpedo::callback, this, _1, _2);
	server.setCallback(f);

	cv::Scalar torpedo_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar enclosing_rectangle_color(149, 255, 23);
	cv::Scalar contour_color(255, 0, 0);

	cv::Mat blue_filtered;
	cv::Mat image_hsv;
	cv::Mat image_thresholded;
	cv::Mat image_marked;
	std::vector<std::vector<cv::Point> > contours;
	cv::Rect bounding_rectangle;
	geometry_msgs::PointStamped torpedo_point_message;
	torpedo_point_message.header.frame_id = camera_frame_.c_str();
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
			blue_filtered = vision_commons::Filter::blue_filter(image_front, clahe_clip_, clahe_grid_size_, clahe_bilateral_iter_, balanced_bilateral_iter_, denoise_h_);
			if (front_high_h_ > front_low_h_ && front_high_s_ > front_low_s_ && front_high_v_ > front_low_v_)
			{
				cv::cvtColor(blue_filtered, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, front_low_h_, front_high_h_, front_low_s_, front_high_s_, front_low_v_, front_high_v_);
				image_thresholded = vision_commons::Morph::open(image_thresholded, 2 * front_opening_mat_point_ + 1, front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * front_closing_mat_point_ + 1, front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);
				contours = vision_commons::Contour::getBestX(image_thresholded, 4);
				if (contours.size() != 0)
				{
					bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
					if (contours.size() == 2)
					{
						bounding_rectangle = cv::boundingRect(cv::Mat(contours[1]));
					}
					else if (contours.size() == 3)
					{
						cv::Rect bounding_rectangle1 = boundingRect(cv::Mat(contours[0]));
						cv::Rect bounding_rectangle2 = boundingRect(cv::Mat(contours[1]));
						cv::Rect bounding_rectangle3 = boundingRect(cv::Mat(contours[2]));
						if (bounding_rectangle1.contains(bounding_rectangle2.br()) && bounding_rectangle2.contains(bounding_rectangle2.tl()))
						{
							bounding_rectangle = bounding_rectangle2;
						}
						else if ((bounding_rectangle2.contains(bounding_rectangle3.br()) && bounding_rectangle2.contains(bounding_rectangle3.tl())) || (bounding_rectangle1.contains(bounding_rectangle3.br()) && bounding_rectangle1.contains(bounding_rectangle3.tl())))
						{
							bounding_rectangle = bounding_rectangle3;
						}
						else
						{
							if (bounding_rectangle3.br().y + bounding_rectangle3.tl().y > bounding_rectangle2.br().y + bounding_rectangle2.tl().y)
							{
								bounding_rectangle = bounding_rectangle3;
							}
							else
								bounding_rectangle = bounding_rectangle2;
						}
					}
					else if (contours.size() == 4)
					{
						cv::Rect bounding_rectangle1 = boundingRect(cv::Mat(contours[2]));
						cv::Rect bounding_rectangle2 = boundingRect(cv::Mat(contours[3]));
						if (bounding_rectangle1.br().y + bounding_rectangle1.tl().y > bounding_rectangle2.br().y + bounding_rectangle2.tl().y)
							bounding_rectangle = bounding_rectangle1;
						else
							bounding_rectangle = bounding_rectangle2;
					}

					if(contours.size()>0)
						detection_bool.data=true;
					else
						detection_bool.data=false;

					front_x_coordinate.data = pow((bounding_rectangle.br().x - bounding_rectangle.tl().x) / 7526.5, -.92678);
					front_y_coordinate.data = (bounding_rectangle.br().x + bounding_rectangle.tl().x) / 2 - (image_front.size().width) / 2;
					front_z_coordinate.data = ((float)image_bottom.size().height) / 2 - (bounding_rectangle.br().y + bounding_rectangle.tl().y) / 2;
					cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x) / 2, (bounding_rectangle.br().y + bounding_rectangle.tl().y) / 2), 1, torpedo_center_color, 8, 0);
					cv::circle(image_marked, cv::Point(image_bottom.size().width / 2, image_front.size().height / 2), 1, image_center_color, 8, 0);
					cv::rectangle(image_marked, bounding_rectangle.tl(), bounding_rectangle.br(), enclosing_rectangle_color);
					for (int i = 0; i < contours.size(); i++)
						cv::drawContours(image_marked, contours, i, contour_color, 1, 8);
				}
			}
			detection_pub.publish(detection_bool);
			front_x_coordinates_pub.publish(front_x_coordinate);
			front_y_coordinates_pub.publish(front_y_coordinate);
			front_z_coordinates_pub.publish(front_z_coordinate);
			
			blue_filtered_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", blue_filtered).toImageMsg());
			front_thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg());
			ROS_INFO("Torpedo Centre Location (x, y, z) = (%.2f, %.2f, %.2f)", front_x_coordinate.data, front_y_coordinate.data, front_z_coordinate.data);
			front_marked_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_marked).toImageMsg());
		}
		else
			ROS_INFO("Image empty");
		loop_rate.sleep();
		ros::spinOnce();
	}
}
