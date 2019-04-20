#include <gate.h>
#include "base_class.h"

Gate::Gate(){
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

	this->front_image_sub = it.subscribe("/anahita/front_camera/image_raw", 1,
											 &Gate::imageFrontCallback, this);
    // this->bottom_image_sub = it.subscribe("/anahita/bottom_camera/image_raw", 1, &Gate::imageBottomCallback, this);

	this->blue_filtered_pub_front = it.advertise("/gate_task/front/blue_filtered", 1);
	this->front_thresholded_pub = it.advertise("/gate_task/front/thresholded", 1);
	this->canny_pub_front = it.advertise("/gate_task/front/canny", 1);
	this->lines_pub_front = it.advertise("/gate_task/front/lines", 1);
	this->front_marked_pub = it.advertise("/gate_task/front/marked", 1);

    this->blue_filtered_pub_bottom = it.advertise("/gate_task/bottom/blue_filtered", 1);
    this->bottom_thresholded_pub = it.advertise("/gate_task/bottom/thresholded", 1);
    this->front_marked_pub = it.advertise("/gate_task/bottom/marked", 1);

	this->front_x_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/x_coordinate", 1000);
	this->front_y_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/y_coordinate", 1000);
	this->front_z_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/z_coordinate", 1000);
    
	this->task_done_pub = nh.advertise<std_msgs::Bool>("/gate_task/done", 1000);
	this->detection_pub = nh.advertise<std_msgs::Bool>("/detected", 1000);
}

cv::Point2i Gate::rotatePoint(const cv::Point2i &v1, const cv::Point2i &v2, float angle)
{
	if (v1.x > v2.x)
	{
		cv::Point2i v3 = v1 - v2;
		cv::Point2i finalVertex;
		finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
		finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
		finalVertex = finalVertex + v2;
		return finalVertex;
	}
	else
	{
		cv::Point2i v3 = v2 - v1;
		cv::Point2i finalVertex;
		finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
		finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
		finalVertex = finalVertex + v1;
		return finalVertex;
	}
};

void Gate::frontCallback(vision_tasks::gateFrontRangeConfig &config, double level)
{
    this->front_clahe_clip_ = config.clahe_clip;
	this->front_clahe_grid_size_ = config.clahe_grid_size;
	this->front_clahe_bilateral_iter_ = config.clahe_bilateral_iter;
	this->front_balanced_bilateral_iter_ = config.balanced_bilateral_iter;
	this->front_denoise_h_ = config.denoise_h;
	this->front_low_h_ = config.low_h;
	this->front_high_h_ = config.high_h;
	this->front_low_s_ = config.low_s;
	this->front_high_s_ = config.high_s;
	this->front_low_v_ = config.low_v;
	this->front_high_v_ = config.high_v;
	this->front_closing_mat_point_ = config.closing_mat_point;
	this->front_closing_iter_ = config.closing_iter;
	this->front_canny_threshold_low_ = config.canny_threshold_low;
	this->front_canny_threshold_high_ = config.canny_threshold_high;
	this->front_canny_kernel_size_ = config.canny_kernel_size;
	this->front_hough_threshold_ = config.hough_threshold;
	this->front_hough_minline_ = config.hough_minline;
	this->front_hough_maxgap_ = config.hough_maxgap;
	this->front_hough_angle_tolerance_ = config.hough_angle_tolerance;
	this->front_gate_distance_tolerance_ = config.gate_distance_tolerance;
	this->front_gate_angle_tolerance_ = config.gate_angle_tolerance;
}

void Gate::bottomCallback(vision_tasks::gateBottomRangeConfig &config, double level)
{
    this->bottom_clahe_clip_ = config.clahe_clip;
    this->bottom_clahe_grid_size_ = config.clahe_grid_size;
    this->bottom_clahe_bilateral_iter_ = config.clahe_bilateral_iter;
    this->bottom_balanced_bilateral_iter_ = config.balanced_bilateral_iter;
    this->bottom_denoise_h_ = config.denoise_h;
    this->bottom_low_h_ = config.low_h;
    this->bottom_high_h_ = config.high_h;
    this->bottom_low_s_ = config.low_s;
    this->bottom_high_s_ = config.high_s;
    this->bottom_low_v_ = config.low_v;
    this->bottom_high_v_ = config.high_v;
    this->bottom_closing_mat_point_ = config.closing_mat_point;
    this->bottom_closing_iter_ = config.closing_iter;
}

void Gate::bottomTaskHandling(bool status) {
	if(status)
	{
		spin_thread_bottom = new boost::thread(&Gate::spinThreadBottom, this); 
	}
	else 
	{
		task_done = true;
        spin_thread_bottom->join();
		std::cout << "Bottom Task Handling function over" << std::endl;	
	}
}

void Gate::spinThreadBottom()
{
	dynamic_reconfigure::Server<vision_tasks::gateBottomRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::gateBottomRangeConfig>::CallbackType f_bottom;
	f_bottom = boost::bind(&Gate::bottomCallback, this, _1, _2);
	server.setCallback(f_bottom);

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
		if (task_done) {
			task_done = false;
			break;
		}
        if (!image_bottom.empty())
        {
			image_bottom.copyTo(image_marked);
			blue_filtered = vision_commons::Filter::blue_filter(image_bottom, bottom_clahe_clip_,
									bottom_clahe_grid_size_, bottom_clahe_bilateral_iter_,
									bottom_balanced_bilateral_iter_, bottom_denoise_h_);
			if (bottom_high_h_ > bottom_low_h_ && bottom_high_s_ > bottom_low_s_ &&
								 bottom_high_v_ > bottom_low_v_)
			{
				cv::cvtColor(blue_filtered, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, bottom_low_h_, 
												bottom_high_h_, bottom_low_s_, bottom_high_s_,
												 bottom_low_v_, bottom_high_v_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 
												2 * bottom_closing_mat_point_ + 1, bottom_closing_mat_point_,
												 bottom_closing_mat_point_, bottom_closing_iter_);
				contours = vision_commons::Contour::getBestX(image_thresholded, 1);
				if (contours.size() != 0)
				{
				bounding_rectangle = cv::minAreaRect(cv::Mat(contours[0]));
				pipe_point_message.header.stamp = ros::Time();
				pipe_point_message.point.x = (image_bottom.size().height) / 2 - bounding_rectangle.center.y;
				pipe_point_message.point.y = bounding_rectangle.center.x - (image_bottom.size().width) / 2;
				pipe_point_message.point.z = 0.0;
				task_done_message.data = pipe_point_message.point.x < 0;
				bounding_rectangle.points(vertices2f);
				for (int i = 0; i < 4; ++i)
				{
					vertices[i] = vertices2f[i];
				}
				cv::circle(image_marked, bounding_rectangle.center, 1, pipe_center_color, 8, 0);
				cv::circle(image_marked, cv::Point(image_bottom.size().width / 2,
									 image_bottom.size().height / 2), 1, image_center_color, 8, 0);
				cv::fillConvexPoly(image_marked, vertices, 4, bounding_rectangle_color);
				}
			}
			blue_filtered_pub_bottom.publish(cv_bridge::CvImage(pipe_point_message.header, 
														"bgr8", blue_filtered).toImageMsg());
			bottom_thresholded_pub.publish(cv_bridge::CvImage(pipe_point_message.header, 
														"mono8", image_thresholded).toImageMsg());
			bottom_coordinates_pub.publish(pipe_point_message);
			//ROS_INFO("Pipe Center (x, y) = (%.2f, %.2f)", pipe_point_message.point.x, pipe_point_message.point.y);
			task_done_pub.publish(task_done_message);
			ROS_INFO("Task done (bool) = %s", task_done_message.data ? "true" : "false");
			bottom_marked_pub.publish(cv_bridge::CvImage(pipe_point_message.header,
														 "bgr8", image_marked).toImageMsg());
        }
        else
            ROS_INFO("Image empty");
        ros::spinOnce();
    }
}

void Gate::frontTaskHandling(bool status) {
	if(status)
	{
		spin_thread_front = new boost::thread(&Gate::spinThreadFront, this); 
	}
	else 
	{
		task_done = true;
        spin_thread_front->join();
		std::cout << "Front Task Handling function over" << std::endl;	
	}
}

void Gate::spinThreadFront()
{
	dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig>::CallbackType f_front;
	f_front = boost::bind(&Gate::frontCallback, this, _1, _2);
	server.setCallback(f_front);

	cv::Scalar gate_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar hough_line_color(0, 164, 253);
	cv::Scalar horizontal_line_color(253, 0, 127);
	cv::Scalar vertical_line_color(0, 253, 0);

	cv::Mat blue_filtered;
	cv::Mat image_hsv;
	cv::Mat image_thresholded;
	cv::Mat image_gray;
	cv::Mat image_canny;
	cv::Mat image_lines;
	cv::Mat image_marked;

	bool found = false;
	std_msgs::Bool detection_bool;
	geometry_msgs::PointStamped gate_point_message;
	gate_point_message.header.frame_id = camera_frame_.c_str();

	while (ros::ok())
	{
		if (task_done) {
			task_done = false;
			break;
		}
		if (!image_front.empty())
		{
			std::vector<cv::Vec4i> lines;
			std::vector<cv::Vec4i> lines_filtered;
			std::vector<double> angles;
			cv::Point2i pi1;
			cv::Point2i pi2;
			cv::Point2i pj1;
			cv::Point2i pj2;
			cv::Point2i horizontal1(0, 0);
			cv::Point2i horizontal2(0, 0);
			cv::Point2i vertical1(0, 0);
			cv::Point2i vertical2(0, 0);			
			cv::Point2i longest1(0, 0);
			cv::Point2i longest2(0, 0);

			image_front.copyTo(image_marked);
			blue_filtered = vision_commons::Filter::blue_filter(image_front, front_clahe_clip_,
											 front_clahe_grid_size_, front_clahe_bilateral_iter_,
											  front_balanced_bilateral_iter_, front_denoise_h_);
			if (front_high_h_ > front_low_h_ && front_high_s_ > front_low_s_ && front_high_v_ > front_low_v_)
			{
				cv::cvtColor(blue_filtered, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, front_low_h_,
													front_high_h_, front_low_s_, front_high_s_,
													front_low_v_, front_high_v_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 
											2 * front_closing_mat_point_ + 1, front_closing_mat_point_, 
											front_closing_mat_point_, front_closing_iter_);
				cv::cvtColor(image_thresholded, image_gray, CV_GRAY2BGR);
				cv::Canny(image_gray, image_canny, front_canny_threshold_low_, front_canny_threshold_high_,
							 front_canny_kernel_size_);
				image_lines = blue_filtered;
				cv::HoughLinesP(image_thresholded, lines, 1, CV_PI / 180, front_hough_threshold_,
									 front_hough_minline_, front_hough_maxgap_);
				double angle = 0.0;
				for (int i = 0; i < lines.size(); i++)
				{
					cv::Point p1(lines[i][0], lines[i][1]);
					cv::Point p2(lines[i][2], lines[i][3]);
					angle = vision_commons::Geometry::angleWrtY(p1, p2);
					if (angle < front_hough_angle_tolerance_ || 
						abs(angle - 90.0) < front_hough_angle_tolerance_ || 
						abs(180.0 - angle) < front_hough_angle_tolerance_)
					{
						cv::line(image_lines, p1, p2, hough_line_color, 3, CV_AA);
						lines_filtered.push_back(lines[i]);
						angles.push_back(angle);
					}
				}
				for (int i = 0; i < lines_filtered.size(); i++)
				{
					pi1.x = lines_filtered[i][0];
					pi1.y = lines_filtered[i][1];
					pi2.x = lines_filtered[i][2];
					pi2.y = lines_filtered[i][3];
					if (vision_commons::Geometry::distance(pi1, pi2) >
					    vision_commons::Geometry::distance(longest1, longest2))
					{
						longest1.x = pi1.x;
						longest1.y = pi1.y;
						longest2.x = pi2.x;
						longest2.y = pi2.y;
					}
					for (int j = i + 1; j < lines_filtered.size(); j++)
					{
						pj1.x = lines_filtered[j][0];
						pj1.y = lines_filtered[j][1];
						pj2.x = lines_filtered[j][2];
						pj2.y = lines_filtered[j][3];
						if (abs(angles[i] - angles[j] - 90.0) < front_gate_angle_tolerance_)
						{
							double distance1 = vision_commons::Geometry::distance(pi1, pj1);
							double distance2 = vision_commons::Geometry::distance(pi1, pj2);
							double distance3 = vision_commons::Geometry::distance(pi2, pj1);
							double distance4 = vision_commons::Geometry::distance(pi2, pj2);
							if (distance1 < front_gate_distance_tolerance_ ||
								 distance2 < front_gate_distance_tolerance_ || 
								 distance3 < front_gate_distance_tolerance_ ||
								  distance4 < front_gate_distance_tolerance_)
							{
								if (abs(angles[j] - 90.0) < abs(angles[i] - 90.0) &&
								 (vision_commons::Geometry::distance(pj1, pj2) > 
								 	vision_commons::Geometry::distance(horizontal1, horizontal2) 
								 || vision_commons::Geometry::distance(pi1, pi2) >
								 	 vision_commons::Geometry::distance(vertical1, vertical2)))
								{
									horizontal1.x = pj1.x;
									horizontal1.y = pj1.y;
									horizontal2.x = pj2.x;
									horizontal2.y = pj2.y;
									vertical1.x = pi1.x;
									vertical1.y = pi1.y;
									vertical2.x = pi2.x;
									vertical2.y = pi2.y;
									found = true;
								}
								else if (abs(angles[j] - 90.0) > abs(angles[i] - 90.0)
								 && (vision_commons::Geometry::distance(pi1, pi2) >
								 		 vision_commons::Geometry::distance(horizontal1, horizontal2) 
								 || vision_commons::Geometry::distance(pj1, pj2) > 
								 		vision_commons::Geometry::distance(vertical1, vertical2)))
								{
									horizontal1.x = pi1.x;
									horizontal1.y = pi1.y;
									horizontal2.x = pi2.x;
									horizontal2.y = pi2.y;
									vertical1.x = pj1.x;
									vertical1.y = pj1.y;
									vertical2.x = pj2.x;
									vertical2.y = pj2.y;
									found = true;
								}
							}
						}
					}
				}
				gate_point_message.header.stamp = ros::Time();
				if (found)
				{
					gate_point_message.point.x = pow(sqrt(pow(vision_commons::Geometry::distance(horizontal1, horizontal2), 2) + pow(vision_commons::Geometry::distance(vertical1, vertical2), 2)) / 7526.5, -.92678);
					gate_point_message.point.y = (horizontal1.x + horizontal2.x) / 2 - image_front.size().width / 2;
					gate_point_message.point.z = image_front.size().height / 2 - (vertical1.y + vertical2.y) / 2;
					ROS_INFO("Gate Center (x, y, z) = (%.2f, %.2f, %.2f)", gate_point_message.point.x, gate_point_message.point.y, gate_point_message.point.z);
					cv::line(image_marked, horizontal1, horizontal2, horizontal_line_color, 3, CV_AA);
					cv::line(image_marked, vertical1, vertical2, vertical_line_color, 3, CV_AA);
				}
				else
				{
					if (abs(vision_commons::Geometry::angleWrtY(longest1, longest2) - 90.0) < front_hough_angle_tolerance_)
					{
						front_x_coordinate.data = pow((vision_commons::Geometry::distance(longest1, longest2) * 1.068) / 7526.5, -.92678);
						front_y_coordinate.data = (longest1.x + longest2.x) / 2 - image_front.size().width / 2;
						front_z_coordinate.data = image_front.size().height / 2 - (longest1.y + longest2.y) / 2 + 9 * vision_commons::Geometry::distance(longest1, longest2) / 48;
					}
					else
					{
						front_x_coordinate.data = pow((vision_commons::Geometry::distance(longest1, longest2) * 2.848) / 7526.5, -.92678);
						front_y_coordinate.data = (longest1.x + longest2.x) / 2 + 12 * vision_commons::Geometry::distance(longest1, longest2) / 9 - image_front.size().width / 2;
						front_z_coordinate.data = image_front.size().height / 2 - (longest1.y + longest2.y) / 2;
					}
					cv::line(image_marked, longest1, longest2, hough_line_color, 3, CV_AA);
					ROS_INFO("Couldn't find gate, estimated gate center (x, y, z) = (%.2f, %.2f, %.2f)", gate_point_message.point.x, gate_point_message.point.y, gate_point_message.point.z);
				}
				cv::circle(image_marked, cv::Point(gate_point_message.point.y + image_front.size().width / 2, image_front.size().height / 2 - gate_point_message.point.z), 1, gate_center_color, 8, 0);
				cv::circle(image_marked, cv::Point(image_front.size().width / 2, image_front.size().height / 2), 1, image_center_color, 8, 0);
			}
			
			detection_bool.data = found;
			detection_pub.publish(detection_bool);

			front_x_coordinates_pub.publish(front_x_coordinate);
			front_y_coordinates_pub.publish(front_y_coordinate);
			front_z_coordinates_pub.publish(front_z_coordinate);
		
			// blue_filtered_pub_front.publish(cv_bridge::CvImage(gate_point_message.header, "bgr8", blue_filtered).toImageMsg());
			front_thresholded_pub.publish(cv_bridge::CvImage(gate_point_message.header, "mono8", image_thresholded).toImageMsg());
			canny_pub_front.publish(cv_bridge::CvImage(gate_point_message.header, "mono8", image_canny).toImageMsg());
			lines_pub_front.publish(cv_bridge::CvImage(gate_point_message.header, "bgr8", image_lines).toImageMsg());
			front_marked_pub.publish(cv_bridge::CvImage(gate_point_message.header, "bgr8", image_marked).toImageMsg());
		}
		else
			ROS_INFO("Image empty");
		ros::spinOnce();
	}
}

void Gate::imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_img_ptr;
	try
	{
		image_front = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
		// ROS_INFO("Found a new image and stored it in image_front!");
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

void Gate::imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg)
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