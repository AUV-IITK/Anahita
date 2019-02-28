#include <buoy.h>

Buoy::Buoy() : it(nh){
	this->clahe_clip_ = 0.15;
	this->clahe_grid_size_ = 3;
	this->clahe_bilateral_iter_ = 2;
	this->balanced_bilateral_iter_ = 4;
	this->denoise_h_ = 10.0;
	this->low_h_ = 0;
	this->high_h_ = 79;
	this->low_s_ = 0;
	this->high_s_ = 24;
	this->low_v_ = 185;
	this->high_v_ = 255;
	this->opening_mat_point_ = 1;
	this->opening_iter_ = 3;
	this->closing_mat_point_ = 1;
	this->closing_iter_ = 0;
	this->camera_frame_ = "auv-iitk";
	this->current_color = 0;
	// image_transport::ImageTransport it(nh);
	this->blue_filtered_pub = it.advertise("/buoy_task/blue_filtered", 1);
	this->thresholded_pub = it.advertise("/buoy_task/thresholded", 1);
	this->marked_pub = it.advertise("/buoy_task/marked", 1);
	this->x_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/x_coordinate", 1000);
	this->y_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/y_coordinate", 1000);
	this->z_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/z_coordinate", 1000);
	this->detection_pub = nh.advertise<std_msgs::Bool>("/detected", 1000);
}

Buoy::~Buoy() {
	spin_thread->join();
}

void Buoy::switchColor(int color)
{
	if(color > 2)
		std::cerr << "Changing to wrong buoy color, use 0-2 for the the different colors" << std::endl;
	else
	{
		current_color = color;		
		this->low_h_ = this->data_low_h[current_color];
		this->high_h_ = this->data_high_h[current_color];
		this->low_s_ = this->data_low_s[current_color];
		this->high_s_ = this->data_high_s[current_color];
		this->low_v_ = this->data_low_v[current_color];
		this->high_v_ = this->data_high_v[current_color];
		ROS_INFO("Current situation colors: %d %d %d", this->low_h_, this->high_h_, this->low_s_);
	}
	std::cout << "Colour changed successfully" << std::endl;
}

void Buoy::setParams(ros::NodeHandle &nh)
{
    nh.getParam("vision_node/low_h", this->low_h_);
	nh.getParam("vision_node/high_h", this->low_h_);
	nh.getParam("vision_node/low_s", this->low_s_);
	nh.getParam("vision_node/high_s", this->high_s_);
	nh.getParam("vision_node/low_v", this->low_v_);
	nh.getParam("vision_node/high_v", this->high_v_);
	nh.getParam("vision_node/clahe_clip", this->clahe_clip_);
	nh.getParam("vision_node/clahe_grid_size", this->clahe_grid_size_);
	nh.getParam("vision_node/clahe_bilateral_iter", this->clahe_bilateral_iter_);
	nh.getParam("vision_node/balanced_bilateral_iter", this->balanced_bilateral_iter_);
	nh.getParam("vision_node/denoise_h", this->denoise_h_);
}
void Buoy::callback(vision_tasks::buoyRangeConfig &config, double level)
{
	ROS_INFO("Update recieved here");
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
	this->low_h_ = config.low_h;
	this->high_h_ = config.high_h;
	this->low_s_ = config.low_s;
	this->high_s_ = config.high_s;
	this->low_v_ = config.low_v;
	this->high_v_ = config.high_v;
	this->opening_mat_point_ = config.opening_mat_point;
	this->opening_iter_ = config.opening_iter;
	this->closing_mat_point_ = config.closing_mat_point;
	this->closing_iter_ = config.closing_iter;

	/*
	if(config.save_param == true)
	{
		ROS_INFO("Dumping the values to the YAML file");
		config.save_param == false;
		system(SHELLSCRIPT_DUMP_RED_BUOY);
	}
	if(config.load_param == true)
	{
		ROS_INFO("Loading the values from the YAML file");
		config.load_param == false;
		system(SHELLSCRIPT_LOAD_RED_BUOY);
		setParams(nh);
		ROS_INFO("Config low_h = %d, this->low_h = %d", config.low_h, this->low_h_);
	}*/
}

void Buoy::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
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

void Buoy::TaskHandling (bool status) {
	if(status)
	{
		task_on = true;
		spin_thread = new boost::thread(boost::bind(&Buoy::spinThread, this)); 
	}
	else 
	{
		close_task = true;
		task_on = false;
        spin_thread->join();
		image_raw_sub.shutdown();
		std::cout << "Buoy Task Handling function over" << std::endl;	
	}
}

void Buoy::spinThread(){

	this->image_raw_sub = it.subscribe("/front_camera/image_raw", 1, &Buoy::imageCallback, this);

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
	std::vector<double> vec_x(5);
	std::vector<double> vec_y(5);
	std::vector<double> vec_z(5);
	double vec_x_avg, vec_y_avg, vec_z_avg;
	int good_values_num = 0;

	std_msgs::Bool detection_bool;

	while (ros::ok())
	{	
	//	image_ = cv::imread("/home/ironman/G0010612.JPG", CV_LOAD_IMAGE_COLOR);
		if (close_task) {
			break;
		}
		if (!image_.empty())
		{
			image_.copyTo(image_marked);

			int64 t0_ = cv::getTickCount();
			blue_filtered = vision_commons::Filter::blue_filter(image_, clahe_clip_, clahe_grid_size_, clahe_bilateral_iter_, balanced_bilateral_iter_, denoise_h_);
			int64 t1_ = cv::getTickCount();
			// ROS_INFO("Time taken by blue-filter: %lf", (t1_-t0_)/cv::getTickFrequency());

			//blue_filtered = image_;
			if (high_h_ > low_h_ && high_s_ > low_s_ && high_v_ > low_v_)
			{	
				int64 t0 = cv::getTickCount();
				std::cout<<"Colour being detected is: " << current_color << std::endl;
				//cv::cvtColor(blue_filtered, image_hsv, CV_BGR2HSV);
				image_hsv = blue_filtered;
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, low_h_, high_h_, low_s_, high_s_, low_v_, high_v_);
				int64 t1 = cv::getTickCount();
				// ROS_INFO("Time taken by thresholding: %lf", (t1-t0)/cv::getTickFrequency());

				image_thresholded = vision_commons::Morph::open(image_thresholded, 2 * opening_mat_point_ + 1, opening_mat_point_, opening_mat_point_, opening_iter_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * closing_mat_point_ + 1, closing_mat_point_, closing_mat_point_, closing_iter_);

				int64 t2 = cv::getTickCount();
				contours = vision_commons::Contour::getBestX(image_thresholded, 2);
				int64 t3 = cv::getTickCount();
				// ROS_INFO("Time taken by contours: %lf", (t3-t2)/cv::getTickFrequency());

				if (contours.size() != 0)
				{
					int index = 0;
					if(contours.size()>=2)
					{
						int area_largest = contourArea(contours[0]);
						int area_second_largest = contourArea(contours[1]);
						cv::Rect br_largest = cv::boundingRect(contours[0]);
						cv::Rect br_second_largest = cv::boundingRect(contours[1]);
						if(((area_largest-area_second_largest) < (0.8*area_largest)) && (br_largest.br().y < br_second_largest.br().y))
						{
						//	ROS_INFO("Changing to below, area_diff = %d", area_largest-area_second_largest);
							index = 1;
						}
					}
					bounding_rectangle = cv::boundingRect(cv::Mat(contours[index]));
					cv::minEnclosingCircle(contours[index], center[0], radius[0]);
					buoy_point_message.header.stamp = ros::Time();
					ROS_INFO("Value of goodnum = %d", good_values_num);
					if(good_values_num == 0)
					{
						vec_x[0] = vec_x[1] = vec_x[2] = vec_x[3] = vec_x[4] = pow(radius[0] / 7526.5, -.92678);
						vec_y[0] = vec_y[1] = vec_y[2] = vec_y[3] = vec_y[4] = center[0].x - ((float)image_.size().width) / 2;
						vec_z[0] = vec_z[1] = vec_z[2] = vec_z[3] = vec_z[4]= ((float)image_.size().height) / 2 - center[0].y;
						//ROS_INFO("Initialising array: %f %f %f %f %f", vec_x[0], vec_x[1], vec_x[2], vec_x[3], vec_x[4]);
					}
					if(good_values_num>4)
					{
						vec_x_avg = (vec_x[0]+vec_x[1]+vec_x[2]+vec_x[3]+vec_x[4])/5;
						vec_y_avg = (vec_y[0]+vec_y[1]+vec_y[2]+vec_y[3]+vec_y[4])/5;
						vec_z_avg = (vec_z[0]+vec_z[1]+vec_z[2]+vec_z[3]+vec_z[4])/5;
						if(abs(pow(radius[0] / 7526.5, -.92678) - vec_x_avg)<0.2*vec_x_avg)
							vec_x[0] = vec_x[1]; vec_x[1] = vec_x[2]; vec_x[2] = vec_x[3]; vec_x[3] = vec_x[4]; vec_x[4] = pow(radius[0] / 7526.5, -.92678);
						//ROS_INFO("Second array: %f %f %f %f %f", vec_x[0], vec_x[1], vec_x[2], vec_x[3], vec_x[4]);
					}
					x_coordinate.data = (vec_x[0]+vec_x[1]+vec_x[2]+vec_x[3]+vec_x[4])/5;
 					y_coordinate.data = center[0].x - ((float)image_.size().width) / 2;
					z_coordinate.data = ((float)image_.size().height) / 2 - center[0].y;
					ROS_INFO("Buoy Location (x, y, z) = (%.2f, %.2f, %.2f)", x_coordinate.data, y_coordinate.data, z_coordinate.data);
					ROS_INFO("Maximum Contour area: %.2f", contourArea(contours[index]));
					if(contourArea(contours[index]) > 1000 && abs(y_coordinate.data) < 235 && abs(z_coordinate.data) < 310) 
					{
						detection_bool.data = true;
						good_values_num++;
					}
					else
						detection_bool.data = false;							
					detection_pub.publish(detection_bool);

					cv::circle(image_marked, cv::Point((bounding_rectangle.br().x + bounding_rectangle.tl().x) / 2, (bounding_rectangle.br().y + bounding_rectangle.tl().y) / 2), 1, buoy_center_color, 8, 0);
					cv::circle(image_marked, cv::Point(image_.size().width / 2, image_.size().height / 2), 1, image_center_color, 8, 0);
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
					good_values_num = 0;
				}	
			}
			if(detection_bool.data)
				ROS_INFO("Detection switch is: 1");
			else
				ROS_INFO("Detection switch is: 0");
			// blue_filtered_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "bgr8", blue_filtered).toImageMsg());
			thresholded_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "mono8", image_thresholded).toImageMsg());
			x_coordinates_pub.publish(x_coordinate);
			y_coordinates_pub.publish(y_coordinate);
			bool enable_pressure = false;
			nh.getParam("/enable_pressure", enable_pressure);
			if (!enable_pressure) {
				z_coordinates_pub.publish(z_coordinate);
			}
			marked_pub.publish(cv_bridge::CvImage(buoy_point_message.header, "bgr8", image_marked).toImageMsg());
		}
		else
		{
			ROS_INFO("Image empty");
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}
