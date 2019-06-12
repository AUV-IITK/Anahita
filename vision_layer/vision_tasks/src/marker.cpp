#include <marker.h>

Marker::Marker() {
	this->loadParams ();
	this->front_roi_pub = it.advertise("/anahita/roi", 1);
}

void Marker::loadParams () {
	nh.getParam("/anahita/vision/marker/b_min", front_low_b_);
	nh.getParam("/anahita/vision/marker/b_max", front_high_b_);
	nh.getParam("/anahita/vision/marker/g_min", front_low_g_);
	nh.getParam("/anahita/vision/marker/g_max", front_high_g_);
	nh.getParam("/anahita/vision/marker/r_min", front_low_r_);
	nh.getParam("/anahita/vision/marker/r_max", front_high_r_);
	nh.getParam("/anahita/vision/marker/closing_mat_point", front_closing_mat_point_);
	nh.getParam("/anahita/vision/marker/closing_iter", front_closing_iter_);
	nh.getParam("/anahita/vision/marker/opening_mat_point", front_opening_mat_point_);
	nh.getParam("/anahita/vision/marker/opening_iter", front_opening_iter_);
	nh.getParam("/anahita/vision/marker/bilateral_iter", front_bilateral_iter_);
}

void Marker::spinThreadFront()
{
	cv::Mat temp_src;
	std::vector<cv::Point> largest_contour;
	cv::Rect bound_rect;
	cv::Scalar bound_rect_color(255, 255, 255);
	cv::Point bound_rect_center;
	sensor_msgs::ImagePtr front_image_marked_msg;
	sensor_msgs::ImagePtr front_image_thresholded_msg;
	ros::Rate loop_rate(15);

	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_front.empty()) {
			ROS_INFO("Found Image");
            vision_mutex.lock();
			temp_src = image_front.clone();
            vision_mutex.unlock();
			vision_commons::Filter::bilateral(temp_src, front_bilateral_iter_);
			image_front_thresholded = vision_commons::Threshold::threshold(temp_src, front_low_b_, front_high_b_,
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
			bound_rect = cv::boundingRect(cv::Mat(largest_contour));
			cv::rectangle(temp_src, bound_rect.tl(), bound_rect.br(), bound_rect_color, 2, 8, 0);
			bound_rect_center.x = ((bound_rect.br()).x + (bound_rect.tl()).x) / 2;
			bound_rect_center.y = ((bound_rect.tl()).y + (bound_rect.br()).y) / 2;

			cv::circle(temp_src, bound_rect_center, 5, cv::Scalar(0, 250, 0), -1, 8, 1);
			cv::circle(temp_src, cv::Point(temp_src.cols/2, temp_src.rows/2), 4, cv::Scalar(150, 150, 150), -1, 8, 0);

			front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg();
        	front_marked_pub.publish(front_image_marked_msg);

			front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg();
        	front_thresholded_pub.publish(front_image_thresholded_msg);
			front_roi_pub.publish(front_image_thresholded_msg);

			front_x_coordinate.data = 0;
			front_y_coordinate.data = bound_rect_center.x - temp_src.cols/2;
			front_z_coordinate.data = temp_src.rows/2 - bound_rect_center.y;

			front_x_coordinate_pub.publish(front_x_coordinate);
			front_y_coordinate_pub.publish(front_y_coordinate);
			front_z_coordinate_pub.publish(front_z_coordinate);
		}
		else {
			ROS_INFO("Image empty");
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}
