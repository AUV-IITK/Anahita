#include <pinger.h>

Pinger::Pinger () {
    loadParams ();
    front_roi_pub = it.advertise("/anahita/roi", 1);
    front_service = nh.advertiseService("/anahita/pinger_front_target", &Pinger::frontTarget, this);
    bottom_service = nh.advertiseService("/anahita/pinger_bottom_target", &Pinger::bottomTarget, this);
}

bool Pinger::bottomTarget (master_layer::PingerBottomTarget::Request &req,
                           master_layer::PingerBottomTarget::Response &resp) {
    resp.success = bottom_visible;
    return true;
}

bool Pinger::frontTarget (master_layer::PingerFrontTarget::Request &req,
                          master_layer::PingerFrontTarget::Response &resp) {
    resp.success = front_visible;
    return true;
}

void Pinger::loadParams () {
    nh.getParam("/anahita/vision/pinger/b_min", front_low_b_);
    nh.getParam("/anahita/vision/pinger/b_max", front_high_b_);
    nh.getParam("/anahita/vision/pinger/g_min", front_low_g_);
    nh.getParam("/anahita/vision/pinger/g_max", front_high_g_);
    nh.getParam("/anahita/vision/pinger/r_min", front_low_r_);
    nh.getParam("/anahita/vision/pinger/r_max", front_high_r_);
    nh.getParam("/anahita/vision/pinger/closing_mat_point", front_closing_mat_point_);
    nh.getParam("/anahita/vision/pinger/closing_iter", front_closing_iter_);
    nh.getParam("/anahita/vision/pinger/opening_mat_point", front_opening_mat_point_);
    nh.getParam("/anahita/vision/pinger/opening_iter", front_opening_iter_);
    nh.getParam("/anahita/vision/pinger/bilateral_iter", front_bilateral_iter_);
}

cv::Point Pinger::getContourCenter (const std::vector<cv::Point>& contour) {
    ROS_ASSERT (cv::contourArea (contour, false) > 0);
    cv::Rect bound_rect = cv::boundingRect (cv::Mat(contour));
    return cv::Point (((bound_rect.br()).x + (bound_rect.tl()).x)/2,
                        ((bound_rect.tl()).y + (bound_rect.br()).y)/2);
}

void Pinger::spinThreadBottom () {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat temp_src;
    ros::Rate loop_rate (20);
    cv::Point contour_center;
    sensor_msgs::ImagePtr bottom_image_marked_msg;
    sensor_msgs::ImagePtr bottom_image_thresholded_msg;

    while (ros::ok()) {
        if (close_task) {
            close_task = false;
            break;
        }
        if (!image_bottom.empty()) {
            temp_src = image_bottom.clone();
            vision_commons::Filter::bilateral(temp_src, front_bilateral_iter_);
            image_bottom_thresholded = vision_commons::Threshold::threshold(temp_src, front_low_b_, front_high_b_,
                                                            front_low_g_, front_high_g_, front_low_r_, front_high_r_);
            vision_commons::Morph::open(image_bottom_thresholded, 2*front_opening_mat_point_ + 1,
                                        front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
            vision_commons::Morph::close(image_bottom_thresholded, 2*front_closing_mat_point_ + 1,
                                         front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);

            cv::findContours (image_bottom_thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
            contours = vision_commons::Contour::filterContours (contours, bottom_size_thres);
            if (!contours.size()) {
                ROS_ERROR("No contours"); 
                bottom_visible = false; 
                continue;
            }
            else { bottom_visible = true; } 
            vision_commons::Contour::sortFromBigToSmall (contours);
            contour_center = getContourCenter (contours[0]);

            bottom_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg();
            bottom_marked_pub.publish(bottom_image_marked_msg);

            bottom_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg();
            bottom_thresholded_pub.publish(bottom_image_thresholded_msg);
        
            bottom_x_coordinate.data = 0;
            bottom_y_coordinate.data = contour_center.x - temp_src.cols/2;
            bottom_z_coordinate.data = contour_center.y - temp_src.rows/2;

            std::cout << "Center of Pinger: " <<  bottom_y_coordinate.data << ", " << bottom_z_coordinate.data << std::endl;

            bottom_x_coordinate_pub.publish(bottom_x_coordinate);
            bottom_y_coordinate_pub.publish(bottom_y_coordinate);
            bottom_z_coordinate_pub.publish(bottom_z_coordinate);

        }
        else ROS_INFO ("Bottom Image Empty");
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void Pinger::spinThreadFront () {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat temp_src;
    ros::Rate loop_rate (20);
    cv::Point contour_center;
    sensor_msgs::ImagePtr front_image_marked_msg;
    sensor_msgs::ImagePtr front_image_thresholded_msg;

    while (ros::ok()) {
        if (close_task) {
            close_task = false;
            break;
        }
        if (!image_front.empty()) {
            vision_mutex.lock();
            temp_src = image_front.clone();
            vision_mutex.unlock();
            vision_commons::Filter::bilateral(temp_src, front_bilateral_iter_);
            image_front_thresholded = vision_commons::Threshold::threshold(temp_src, front_low_b_, front_high_b_,
                                                            front_low_g_, front_high_g_, front_low_r_, front_high_r_);
            vision_commons::Morph::open(image_front_thresholded, 2*front_opening_mat_point_ + 1,
                                        front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
            vision_commons::Morph::close(image_front_thresholded, 2*front_closing_mat_point_ + 1,
                                         front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);

            cv::findContours (image_front_thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
            contours = vision_commons::Contour::filterContours (contours, front_size_thres);
            if (!contours.size()) {
                ROS_ERROR("No contours"); 
                front_visible = false;
                continue;
            }
            else { front_visible = true; }
            vision_commons::Contour::sortFromBigToSmall (contours);
            contour_center = getContourCenter (contours[0]);

            front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg();
            front_marked_pub.publish(front_image_marked_msg);

            front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg();
            front_thresholded_pub.publish(front_image_thresholded_msg);
        
            front_x_coordinate.data = 0;
            front_y_coordinate.data = contour_center.x - temp_src.cols/2;
            front_z_coordinate.data = contour_center.y - temp_src.rows/2;

            std::cout << "Center of Pinger: " <<  front_y_coordinate.data << ", " << front_z_coordinate.data << std::endl;

            front_x_coordinate_pub.publish(front_x_coordinate);
            front_y_coordinate_pub.publish(front_y_coordinate);
            front_z_coordinate_pub.publish(front_z_coordinate);

        }
        else ROS_INFO ("Front Image Empty");
        loop_rate.sleep();
        ros::spinOnce();
    }
}
