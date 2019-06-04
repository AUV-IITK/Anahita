#include <grabber.h>

Grabber::Grabber() {
	this->loadParams ();
	this->front_roi_pub = it.advertise("/anahita/roi", 1);
}

Grabber::~Grabber() {}

void Grabber::loadParams () {
	nh.getParam("/anahita/vision/grabber/b_min", front_low_b_);
	nh.getParam("/anahita/vision/grabber/b_max", front_high_b_);
	nh.getParam("/anahita/vision/grabber/g_min", front_low_g_);
	nh.getParam("/anahita/vision/grabber/g_max", front_high_g_);
	nh.getParam("/anahita/vision/grabber/r_min", front_low_r_);
	nh.getParam("/anahita/vision/grabber/r_max", front_high_r_);
	nh.getParam("/anahita/vision/grabber/closing_mat_point", front_closing_mat_point_);
	nh.getParam("/anahita/vision/grabber/closing_iter", front_closing_iter_);
	nh.getParam("/anahita/vision/grabber/opening_mat_point", front_opening_mat_point_);
	nh.getParam("/anahita/vision/grabber/opening_iter", front_opening_iter_);
	nh.getParam("/anahita/vision/Grabber_test/bilateral_iter", front_bilateral_iter_);
}

void Grabber::spinThreadFront()
{
	cv::Mat image_marked, edges, image_hsv;
	std::vector<cv::Point> largest_contour;
	cv::Rect bound_rect;
    cv::Scalar line_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar edge_color(255, 255, 255);
	cv::Scalar hough_line_color(0, 164, 253);
	cv::Scalar contour_color(255, 0, 0);
	cv::Scalar bound_rect_color(255, 255, 255);
	cv::Point bound_rect_center;
   	std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> lines;   
	cv::Point2f center;
	cv::Point2f grabber_target;
	sensor_msgs::ImagePtr front_image_marked_msg;
	sensor_msgs::ImagePtr front_image_thresholded_msg;
	sensor_msgs::ImagePtr front_edges_msg;
	ros::Rate loop_rate(15);
	float radius = 0;
	float grabber_threshold = 0;
    ROS_INFO("__________________________________________________");
    
	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_front.empty()) {
			nh.getParam("grabber_x", grabber_target.x);
			nh.getParam("grabber_y", grabber_target.y);
			nh.getParam("grabber_threshold", grabber_threshold);

			ROS_INFO("Found Image");
			image_marked = image_front.clone();

			image_front_thresholded = vision_commons::Threshold::threshold(image_front, front_low_b_, front_high_b_,
                                                                           front_low_g_, front_high_g_,
                                                                           front_low_r_, front_high_r_);
            vision_commons::Morph::open(image_front_thresholded, 2 * front_opening_mat_point_ + 1,
                                        front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
            vision_commons::Morph::close(image_front_thresholded, 2 * front_closing_mat_point_ + 1,
                                         front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);
            largest_contour = vision_commons::Contour::getLargestContour(image_front_thresholded);
            if (!largest_contour.size()) {
                ROS_INFO("No contour found");
                continue;
            }
			cv::minEnclosingCircle((cv::Mat)largest_contour, center, radius);
			ROS_INFO("Center of the circle: %f %f, radius: %f", center.x, center.y, radius);
			cv::circle(image_marked, center, radius, cv::Scalar(0, 250, 0), 2, 8, 0);
			cv::circle(image_marked, center, 2, cv::Scalar(255, 250, 0), 2, 8, 0);
			cv::circle(image_marked, grabber_target, 3, cv::Scalar(255, 0, 0), 2, 8, 0);
			
			front_x_coordinate.data = 0;
            front_y_coordinate.data = center.x - image_front.cols/2;
            front_z_coordinate.data = image_front.rows/2 - center.y;
			
						
			double dist = sqrt((grabber_target.x - center.x)*(grabber_target.x - center.x) + (grabber_target.y - center.y)*(grabber_target.y - center.y));

			front_x_coordinate_pub.publish(front_x_coordinate);
            front_y_coordinate_pub.publish(front_y_coordinate);
            front_z_coordinate_pub.publish(front_z_coordinate);


            front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_marked).toImageMsg();
            front_marked_pub.publish(front_image_marked_msg);

            front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg();
            front_thresholded_pub.publish(front_image_thresholded_msg);
		}
		else {
			ROS_INFO("Image empty");
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}

void Grabber::spinThreadBottom () {}
