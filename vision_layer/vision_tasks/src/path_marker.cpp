#include "path_marker.h"

PathMarker::PathMarker() {
	this->loadParams ();
    service = nh.advertiseService("/anahita/marker_angle", &PathMarker::markerAngle, this);
}

void PathMarker::loadParams () {
	nh.getParam("/anahita/vision/path_marker/b_min", bottom_low_b_);
	nh.getParam("/anahita/vision/path_marker/b_max", bottom_high_b_);
	nh.getParam("/anahita/vision/path_marker/g_min", bottom_low_g_);
	nh.getParam("/anahita/vision/path_marker/g_max", bottom_high_g_);
	nh.getParam("/anahita/vision/path_marker/r_min", bottom_low_r_);
	nh.getParam("/anahita/vision/path_marker/r_max", bottom_high_r_);
	nh.getParam("/anahita/vision/path_marker/closing_mat_point", bottom_closing_mat_point_);
	nh.getParam("/anahita/vision/path_marker/closing_iter", bottom_closing_iter_);
	nh.getParam("/anahita/vision/path_marker/opening_mat_point", bottom_opening_mat_point_);
	nh.getParam("/anahita/vision/path_marker/opening_iter", bottom_opening_iter_);
	nh.getParam("/anahita/vision/path_marker/bilateral_iter", bottom_bilateral_iter_);
}

bool PathMarker::markerAngle (master_layer::RequestMarkerAngle::Request &req,
                               master_layer::RequestMarkerAngle::Response &res) {
    res.major_angle = MAJOR;
    res.minor_angle = MINOR;
    return true;
}

void PathMarker::spinThreadBottom()
{
	cv::Mat temp_src;
    cv::Mat canny_edge;
	std::vector<cv::Point> largest_contour;
	cv::Rect bound_rect;
	cv::Scalar bound_rect_color(255, 255, 255);
	cv::Point bound_rect_center;
	sensor_msgs::ImagePtr bottom_image_marked_msg;
	sensor_msgs::ImagePtr bottom_image_thresholded_msg;
	ros::Rate loop_rate(15);
    double angles1_avg = 0;
    double angles2_avg = 0;
    std::vector<double> angles;
    std::vector<double> angles1;
    std::vector<double> angles2;

	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_bottom.empty()) {
			temp_src = image_bottom.clone();
			vision_commons::Filter::bilateral(temp_src, bottom_bilateral_iter_);
			image_bottom_thresholded = vision_commons::Threshold::threshold(temp_src, bottom_low_b_, bottom_high_b_,
																			bottom_low_g_, bottom_high_g_,
																			bottom_low_r_, bottom_high_r_);
			vision_commons::Morph::open(image_bottom_thresholded, 2 * bottom_opening_mat_point_ + 1, 
										bottom_opening_mat_point_, bottom_opening_mat_point_, bottom_opening_iter_);
			vision_commons::Morph::close(image_bottom_thresholded, 2 * bottom_closing_mat_point_ + 1, 
										bottom_closing_mat_point_, bottom_closing_mat_point_, bottom_closing_iter_);

            std::vector<cv::Vec4i> lines;
            cv::Canny(image_bottom_thresholded, canny_edge, 50, 200, 3);
            HoughLinesP(canny_edge, lines, 1, CV_PI/180, 50, 50, 10 );
            
            for( size_t i = 0; i < lines.size(); i++ )
            {
                double theta = atan(static_cast<double>(lines[i][2] - lines[i][0]) / (lines[i][1] - lines[i][3])) * 180.0 / 3.14159;
                angles.push_back(theta);
                cv::line(temp_src, cv::Point(lines[i][0], lines[i][1]), 
                        cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 1, CV_AA);
            }

            std::sort(angles.begin(), angles.end());

            for (size_t i = 0; i < lines.size(); i++) {
                if (angles[i] > angles[0] + 37) {
                    angles2.push_back(angles[i]);
                }
                else {
                    angles1.push_back(angles[i]);
                }
            }
            
            if (angles1.size() != 0) {
                angles1_avg = std::accumulate(angles1.begin(), angles1.end(), 0)/angles1.size();
            }
            if (angles2.size() != 0) {
                angles2_avg = std::accumulate(angles2.begin(), angles2.end(), 0)/angles2.size();
            }

            std::cout << "angle 1 avg: " << angles1_avg << std::endl;
            std::cout << "angle 2 avg: " << angles2_avg << std::endl;

            MAJOR = std::abs(angles1_avg) > std::abs(angles2_avg)?angles1_avg : angles2_avg;
            MINOR = std::abs(angles1_avg) > std::abs(angles2_avg)?angles2_avg : angles1_avg;

			largest_contour = vision_commons::Contour::getLargestContour(image_bottom_thresholded);
			bound_rect = cv::boundingRect(cv::Mat(largest_contour));
			cv::rectangle(temp_src, bound_rect.tl(), bound_rect.br(), bound_rect_color, 2, 8, 0);
			bound_rect_center.x = ((bound_rect.br()).x + (bound_rect.tl()).x) / 2;
			bound_rect_center.y = ((bound_rect.tl()).y + (bound_rect.br()).y) / 2;

			cv::circle(temp_src, bound_rect_center, 5, cv::Scalar(0, 250, 0), -1, 8, 1);
			cv::circle(temp_src, cv::Point(temp_src.cols/2, temp_src.rows/2), 4, cv::Scalar(150, 150, 150), -1, 8, 0);

			bottom_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg();
        	bottom_marked_pub.publish(bottom_image_marked_msg);

			bottom_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_bottom_thresholded).toImageMsg();
        	bottom_thresholded_pub.publish(bottom_image_thresholded_msg);

			bottom_x_coordinate.data = bound_rect_center.y - temp_src.rows/2;
			bottom_y_coordinate.data = bound_rect_center.x - temp_src.cols/2;
			bottom_z_coordinate.data = 0;

			bottom_x_coordinate_pub.publish(bottom_x_coordinate);
			bottom_y_coordinate_pub.publish(bottom_y_coordinate);
			bottom_z_coordinate_pub.publish(bottom_z_coordinate);
		}
		else {
			ROS_INFO("Image empty");
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}
