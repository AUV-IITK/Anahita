#include <marker_dropper.h>

MarkerDropper::MarkerDropper() {
	loadParams ();
}

void MarkerDropper::loadParams () {
	nh.getParam("/anahita/vision/path_marker/b_min", bgr_min[0]);
	nh.getParam("/anahita/vision/path_marker/b_max", bgr_max[0]);
	nh.getParam("/anahita/vision/path_marker/g_min", bgr_min[1]);
	nh.getParam("/anahita/vision/path_marker/g_max", bgr_max[1]);
	nh.getParam("/anahita/vision/path_marker/r_min", bgr_min[2]);
	nh.getParam("/anahita/vision/path_marker/r_max", bgr_max[2]);
	nh.getParam("/anahita/vision/path_marker/closing_mat_point", bottom_closing_mat_point_);
	nh.getParam("/anahita/vision/path_marker/closing_iter", bottom_closing_iter_);
	nh.getParam("/anahita/vision/path_marker/opening_mat_point", bottom_opening_mat_point_);
	nh.getParam("/anahita/vision/path_marker/opening_iter", bottom_opening_iter_);
	nh.getParam("/anahita/vision/path_marker/bilateral_iter", bottom_bilateral_iter_);
}

void MarkerDropper::preProcess (cv::Mat& temp_src) {
    vision_commons::Filter::bilateral(temp_src, bottom_bilateral_iter_);
    image_bottom_thresholded = vision_commons::Threshold::threshold(temp_src, bgr_min, bgr_max);
    vision_commons::Morph::open(image_bottom_thresholded, 2 * bottom_opening_mat_point_ + 1, 
                                bottom_opening_mat_point_, bottom_opening_mat_point_, bottom_opening_iter_);
    vision_commons::Morph::close(image_bottom_thresholded, 2 * bottom_closing_mat_point_ + 1, 
                                bottom_closing_mat_point_, bottom_closing_mat_point_, bottom_closing_iter_);
}

cv::Point MarkerDropper::findCenter () {
    static std::vector<std::vector<cv::Point> > contours;
    static std::vector<cv::Vec4i> hierarchy;
    cv::findContours (image_bottom_thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    contours = vision_commons::Contour::filterContours (contours, 100);
    if (!contours.size()) return cv::Point(-1, -1);
    vision_commons::Contour::sortFromBigToSmall (contours);
    cv::Rect bound_rect = cv::boundingRect (cv::Mat(contours[0]));
    return cv::Point (((bound_rect.br()).x + (bound_rect.tl()).x)/2,
                        ((bound_rect.tl()).y + (bound_rect.br()).y)/2);
}

void MarkerDropper::spinThreadBottom()
{
	cv::Mat temp_src;
    ros::Rate loop_rate(20);
    cv::Point center;

	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_bottom.empty()) {
			temp_src = image_bottom.clone();
            preProcess (temp_src);
            center = findCenter ();
            if (center == cv::Point(-1, -1)) continue;

            bottom_marked_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg());
            bottom_thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg());

            bottom_x_coordinate.data = center.y - temp_src.rows/2;
            bottom_y_coordinate.data = center.x - temp_src.cols/2;
            bottom_z_coordinate.data = 0;

            bottom_x_coordinate_pub.publish(bottom_x_coordinate);
            bottom_y_coordinate_pub.publish(bottom_y_coordinate);
            bottom_z_coordinate_pub.publish(bottom_z_coordinate);

            std::cout << "Center of Pinger: " <<  bottom_y_coordinate.data << ", " << bottom_z_coordinate.data << std::endl;
		}
		else ROS_INFO("Image empty");
		loop_rate.sleep();
		ros::spinOnce();
	}
}
