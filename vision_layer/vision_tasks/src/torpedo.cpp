#include <torpedo.h>

Torpedo::Torpedo(){
	this->loadParams ();
}

Torpedo::~Torpedo() {}

void Torpedo::loadParams(){
	nh.getParam("/anahita/vision/torpedo/b_min", front_low_b_);
	nh.getParam("/anahita/vision/torpedo/b_max", front_high_b_);
	nh.getParam("/anahita/vision/torpedo/g_min", front_low_g_);
	nh.getParam("/anahita/vision/torpedo/g_max", front_high_g_);
	nh.getParam("/anahita/vision/torpedo/r_min", front_low_r_);
	nh.getParam("/anahita/vision/torpedo/r_max", front_high_r_);
	nh.getParam("/anahita/vision/torpedo/closing_mat_point", front_closing_mat_point_);
	nh.getParam("/anahita/vision/torpedo/closing_iter", front_closing_iter_);
	nh.getParam("/anahita/vision/torpedo/opening_mat_point", front_opening_mat_point_);
	nh.getParam("/anahita/vision/torpedo/opening_iter", front_opening_iter_);
	nh.getParam("/anahita/vision/torpedo/bilateral_iter", front_bilateral_iter_);
}

void Torpedo::spinThreadFront(){

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

	sensor_msgs::ImagePtr front_image_marked_msg;
	sensor_msgs::ImagePtr front_image_thresholded_msg;
	ros::Rate loop_rate(25);
	std_msgs::Bool detection_bool;

	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_front.empty())
		{
			//cv::resize(image_front, image_front, cv::Size(640, 480), 0, 0, CV_INTER_LINEAR);
			ROS_INFO("Foundd Image: %d %d", image_front.cols, image_front.rows);
			image_front.copyTo(image_marked);
			cv::cvtColor(image_marked, image_hsv, CV_BGR2HSV);
			image_thresholded = vision_commons::Threshold::threshold(image_hsv, front_low_b_, front_high_b_, front_low_g_, front_high_g_, front_low_r_, front_high_r_);
		    vision_commons::Morph::open(image_thresholded, 2 * front_opening_mat_point_ + 1, front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
			vision_commons::Morph::close(image_thresholded, 2 * front_closing_mat_point_ + 1, front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);
			//contours = vision_commons::Contour::getBestX(image_thresholded, 4);


			front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_marked).toImageMsg();
        	front_marked_pub.publish(front_image_marked_msg);


			front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg();
        	front_thresholded_pub.publish(front_image_thresholded_msg);

			
		}
		else
			ROS_INFO("Image empty");
		loop_rate.sleep();
		ros::spinOnce();
	}
}

void Torpedo::spinThreadBottom () {}
