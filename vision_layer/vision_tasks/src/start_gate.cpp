#include <start_gate.h>

StartGate::StartGate() {
    this->loadParams();
    this->front_roi_pub = it.advertise("/anahita/roi", 1);
}

void StartGate::loadParams() {
    nh.getParam("/anahita/vision/start_gate/b_min", front_low_b_);
    nh.getParam("/anahita/vision/start_gate/b_max", front_high_b_);
    nh.getParam("/anahita/vision/start_gate/g_min", front_low_g_);
    nh.getParam("/anahita/vision/start_gate/g_max", front_high_g_);
    nh.getParam("/anahita/vision/start_gate/r_min", front_low_r_);
    nh.getParam("/anahita/vision/start_gate/r_max", front_high_r_);
    nh.getParam("/anahita/vision/start_gate/closing_mat_point", front_closing_mat_point_);
    nh.getParam("/anahita/vision/start_gate/closing_iter", front_closing_iter_);
    nh.getParam("/anahita/vision/start_gate/opening_mat_point", front_opening_mat_point_);
    nh.getParam("/anahita/vision/start_gate/opening_iter", front_opening_iter_);
    nh.getParam("/anahita/vision/start_gate/bilateral_iter", front_bilateral_iter_);
}

bool contour_cmp (std::vector<cv::Point> contour_a, std::vector<cv::Point> contour_b) {
    return cv::contourArea (contour_a, false) > cv::contourArea (contour_b, false);
}

double dist (cv::Point a, cv::Point b) {
    double dist_ = (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
    return dist_;
}

void filterContours (const std::vector<std::vector<cv::Point> >& contours, std::vector<int>& idx, double threshold) {
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea (contours[i], false) < threshold) continue;
        idx.push_back (i);
    }
}

cv::Point StartGate::findGateCenter (cv::Mat& thres_img) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours (thres_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    if (contours.size() > 1) std::sort (contours.begin(), contours.end(), contour_cmp);
    else ROS_ERROR ("No. of contours should be more than one");

    cv::Point center;

    std::vector<int> idx;
    filterContours (contours, idx, 75);

    if (!idx.size()) return cv::Point(-1, -1);
    if (idx.size() == 1) {
        cv::Rect bound_rect;
        bound_rect = cv::boundingRect (cv::Mat(contours[idx[0]]));
        center.x = ((bound_rect.br()).x + (bound_rect.tl()).x)/2;
        center.y = ((bound_rect.tl()).y + (bound_rect.br()).y)/2;
    }
    else if (idx.size() == 2) {
        cv::Rect bigger_rect;
        cv::Rect smaller_rect;
        bigger_rect = cv::boundingRect (cv::Mat (contours[idx[0]]));
        smaller_rect = cv::boundingRect (cv::Mat (contours[idx[1]]));
        center.x = ((bigger_rect.br()).x + (bigger_rect.tl()).x)/4 + ((smaller_rect.br()).x + (smaller_rect.tl()).x)/4;
        center.y = ((bigger_rect.br()).y + (bigger_rect.tl()).y)/4 + ((smaller_rect.br()).y + (smaller_rect.tl()).y)/4;
    }
    else if (idx.size() >= 3) {
        std::vector<cv::Rect> rects;
        
        for (int i = 0; i < 3; i++)
            rects.push_back (cv::boundingRect(contours[idx[i]]));

        std::vector<cv::Point> centers(3);
        for (int i = 0; i < 3; i++) {
            centers[i].x = ((rects[i].br()).x + (rects[i].tl()).x)/2;
            centers[i].y = ((rects[i].tl()).y + (rects[i].br()).y)/2;
        }

        int idx_ = 0;
        if (dist (centers[2], centers[1]) < dist (centers[2], centers[0])) idx_ = 1;

        center.x = ((rects[idx_].br()).x + (rects[idx_].tl()).x)/4 + ((rects[2].br()).x + (rects[2].tl()).x)/4 - thres_img.cols/2;
        center.y = thres_img.rows/2 - ((rects[idx_].br()).x + (rects[idx_].tl()).x)/4 + ((rects[2].br()).x + (rects[2].tl()).x)/4;
    }

    return center;
}

void StartGate::spinThreadFront() {
    cv::Mat temp_src;
    std::vector<cv::Point> largest_contour;
    cv::Point center;
    sensor_msgs::ImagePtr front_image_marked_msg;
    sensor_msgs::ImagePtr front_image_thresholded_msg;
    ros::Rate loop_rate(15);

    while (ros::ok())
    {
        if (close_task)
        {
            close_task = false;
            break;
        }
        if (!image_front.empty())
        {
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
            center = findGateCenter (image_front_thresholded);
            if (center == cv::Point(-1, -1)) ROS_ERROR ("Gate center not found!!"); continue; 
            cv::circle(marked_img, center, 10, cv::Scalar(0, 250, 0), -1, 8, 1);
            cv::circle(marked_img, cv::Point(temp_src.cols/2, temp_src.rows/2), 4, cv::Scalar(150, 150, 150), -1, 8, 0);

            front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg();
            front_marked_pub.publish(front_image_marked_msg);

            front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg();
            front_thresholded_pub.publish(front_image_thresholded_msg);
            front_roi_pub.publish(front_image_thresholded_msg);

            front_x_coordinate.data = 0;
            front_y_coordinate.data = center.x - temp_src.cols/2;
            front_z_coordinate.data = temp_src.rows/2 - center.y;

            std::cout << "Center of Gate: " <<  front_y_coordinate.data << ", " << front_z_coordinate.data << std::endl;

            front_x_coordinate_pub.publish(front_x_coordinate);
            front_y_coordinate_pub.publish(front_y_coordinate);
            front_z_coordinate_pub.publish(front_z_coordinate);
        }
        else
        {
            ROS_INFO("Image empty");
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}
