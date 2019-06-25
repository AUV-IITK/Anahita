#include <buoy.h>

Buoy::Buoy () {
    loadParams();
    front_roi_pub = it.advertise ("/anahita/roi", 1);
    features_pub = nh.advertise<std_msgs::Int32MultiArray>("/anahita/features", 10);
}

void Buoy::loadParams () {
    nh.getParam("/anahita/vision/buoy/b_min", front_low_b_);
    nh.getParam("/anahita/vision/buoy/b_max", front_high_b_);
    nh.getParam("/anahita/vision/buoy/g_min", front_low_g_);
    nh.getParam("/anahita/vision/buoy/g_max", front_high_g_);
    nh.getParam("/anahita/vision/buoy/r_min", front_low_r_);
    nh.getParam("/anahita/vision/buoy/r_max", front_high_r_);
    nh.getParam("/anahita/vision/buoy/closing_mat_point", front_closing_mat_point_);
    nh.getParam("/anahita/vision/buoy/closing_iter", front_closing_iter_);
    nh.getParam("/anahita/vision/buoy/opening_mat_point", front_opening_mat_point_);
    nh.getParam("/anahita/vision/buoy/opening_iter", front_opening_iter_);
    nh.getParam("/anahita/vision/buoy/bilateral_iter", front_bilateral_iter_);
}

cv::Mat Buoy::preprocess (const cv::Mat& temp_src) {
    cv::Mat thres_img;
    vision_commons::Filter::bilateral(temp_src, front_bilateral_iter_);
    thres_img = vision_commons::Threshold::threshold(temp_src, front_low_b_, front_high_b_,
                                    front_low_g_, front_high_g_, front_low_r_, front_high_r_);
    vision_commons::Morph::open(thres_img, 2 * front_opening_mat_point_ + 1,
                        front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
    vision_commons::Morph::close(thres_img, 2 * front_closing_mat_point_ + 1,
                        front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);
    return thres_img;
}

cv::Point get_center (const cv::Rect& rect) {
    cv::Point center;
    center.x = (rect.br().x + rect.tl().x)/2;
    center.y = (rect.br().y + rect.tl().y)/2;
    return center;
}

void Buoy::extractFeatures (const cv::Mat& thres_img) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours (thres_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    if (contours.size())
        contours = vision_commons::Contour::filterContours(contours, 200);
    if (contours.size() == 0) return;
    vision_commons::Contour::sortFromBigToSmall (contours);

    std::vector<cv::Point> contour;
    contour = contours[0];

    cv::Rect bound_rect = cv::boundingRect (cv::Mat(contour));
    cv::Rect bound_center = get_center (bound_rect);

    static int x = -1;
    static int y = -1;
    static int l_x = -1;
    static int l_y = -1;

    x = bound_center.x;
    y = bound_center.y;
    l_x = std::abs(bound_rect.br().x - bound_rect.tl().x);
    l_y = std::abs(bound_rect.br().y - bound_rect.tl().y);

    feature_msg.data.push_back(x); feature_msg.data.push_back(y);
    feature_msg.data.push_back(l_x); feature_msg.data.push_back(l_y);
    features_pub.publish(feature_msg);
}

void Buoy::spinThreadFront () {

    ROS_INFO ("vision: buoy activated");
    cv::Mat temp_src;
	while (ros::ok()) {	
		if (close_task) {
            close_task = false;
			break;
		}
		if (!image_front.empty()) {
			image_front.copyTo(temp_src);
            image_front_thresholded = preprocess (temp_src);
            extractFeatures (image_front_thresholded);
		}
		else ROS_INFO("Image empty");
		loop_rate.sleep();
		ros::spinOnce();
	}
}
