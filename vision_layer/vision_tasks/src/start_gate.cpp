#include <start_gate.h>

StartGate::StartGate() {
    loadParams();
    front_roi_pub = it.advertise("/anahita/roi", 1);
    features_pub = nh.advertise<std_msgs::Int32MultiArray>("/anahita/features", 10, true);
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

// bool contour_cmp (std::vector<cv::Point> contour_a, std::vector<cv::Point> contour_b) {
//     return cv::contourArea (contour_a, false) > cv::contourArea (contour_b, false);
// }

double dist (cv::Point a, cv::Point b) {
    double dist_ = (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
    return dist_;
}

// void filterContours (const std::vector<std::vector<cv::Point> >& contours, std::vector<int>& idx, double threshold) {
//     for (int i = 0; i < contours.size(); i++) {
//         if (cv::contourArea (contours[i], false) < threshold) continue;
//         idx.push_back (i);
//     }
// }

cv::Point StartGate::findGateCenter (cv::Mat& thres_img) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours (thres_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    contours = vision_commons::Contour::filterContours(contours, 100);
    if (contours.size() < 1) return cv::Point(-1, -1);
    vision_commons::Contour::sortFromBigToSmall(contours);

    cv::Point center(-1, -1);

    if (contours.size() == 1) {
        cv::Rect bound_rect;
        bound_rect = cv::boundingRect (cv::Mat(contours[0]));
        center.x = ((bound_rect.br()).x + (bound_rect.tl()).x)/2;
        center.y = ((bound_rect.tl()).y + (bound_rect.br()).y)/2;
    }
    else if (contours.size() == 2) {
        cv::Rect bigger_rect;
        cv::Rect smaller_rect;
        bigger_rect = cv::boundingRect (cv::Mat (contours[0]));
        smaller_rect = cv::boundingRect (cv::Mat (contours[1]));
        center.x = ((bigger_rect.br()).x + (bigger_rect.tl()).x)/4 + ((smaller_rect.br()).x + (smaller_rect.tl()).x)/4;
        center.y = ((bigger_rect.br()).y + (bigger_rect.tl()).y)/2 - 0.3*((smaller_rect.br()).y - (smaller_rect.tl()).y);
    }
    else if (contours.size() >= 3) {
        std::vector<cv::Rect> rects;
        
        for (int i = 0; i < 3; i++)
            rects.push_back (cv::boundingRect(contours[i]));

        std::vector<cv::Point> centers(3);
        for (int i = 0; i < 3; i++) {
            centers[i].x = ((rects[i].br()).x + (rects[i].tl()).x)/2;
            centers[i].y = ((rects[i].tl()).y + (rects[i].br()).y)/2;
        }

        int idx_ = 0;
        if (dist (centers[2], centers[1]) < dist (centers[2], centers[0])) idx_ = 1;

        int mean_l = 0.5*((rects[0].br()).y - (rects[0].tl()).y) + 0.5*((rects[0].br()).y - (rects[0].tl()).y);

        center.x = ((rects[idx_].br()).x + (rects[idx_].tl()).x)/4 + ((rects[2].br()).x + (rects[2].tl()).x)/4;
        center.y = ((rects[0].br()).x + (rects[0].tl()).x)/4 + ((rects[1].br()).x + (rects[1].tl()).x)/4 - 0.3*mean_l;
    }

    return center;
}

bool cmp_x (cv::Point a, cv::Point b) {
    return a.x < b.x;
}

cv::Point2d rect_center (cv::Rect2d bound_rect) {
    cv::Point2d center;
    center.x = ((bound_rect.br()).x + (bound_rect.tl()).x)/2;
    center.y = ((bound_rect.tl()).y + (bound_rect.br()).y)/2;
    return center;
}

bool in_range (float a, float b, float range) {
    if (std::abs(a-b) <= range) return true;
    return false;
}

void fill_msg (std_msgs::Int32MultiArray &msg, int f0, int f1, int f2, int f3, int f4, 
               int f5, int f6, int f7, int f8, int f9, int f10, int f11, int f12, int f13, int f14) {
    msg.data.clear();
    msg.data.push_back(f0); msg.data.push_back(f1); msg.data.push_back(f2);
    msg.data.push_back(f3); msg.data.push_back(f4); msg.data.push_back(f5);
    msg.data.push_back(f6); msg.data.push_back(f7); msg.data.push_back(f8);
    msg.data.push_back(f9); msg.data.push_back(f10); msg.data.push_back(f11);
    msg.data.push_back(f12); msg.data.push_back(f13); msg.data.push_back(f14);
}

void StartGate::extractFeatures (const cv::Mat& thres_img) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours (thres_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    contours = vision_commons::Contour::filterContours(contours, 50);
    if (contours.size() < 2) return;
    vision_commons::Contour::sortFromBigToSmall(contours);

    int l_x = -1;
    int l_y = -1;
    int m_x = -1; 
    int m_y = -1;
    int r_x = -1;
    int r_y = -1;
    int l_l = -1;
    int r_l = -1;
    int m_l = -1;
    int l_m_h = -1;
    int m_r_h = -1;
    int l_r_h = -1;
    int l_m_v = -1;
    int m_r_v = -1;
    int l_r_v = -1;

    if (contours.size() == 2) {
        if (in_range(cv::contourArea (contours[1], false), cv::contourArea (contours[0], false), 25)) { // left and right
            cv::Rect2d rect0; cv::Rect2d rect1;
            rect0 = cv::boundingRect (cv::Mat(contours[0]));
            rect1 = cv::boundingRect (cv::Mat(contours[1]));
            cv::Point2d center0; cv::Point2d center1;
            center0 = rect_center (rect0); center1 = rect_center (rect1);
            if (center0.x < center1.x) { // 0 => left
                l_x = center0.x;
                l_y = center0.y;
                r_x = center1.x;
                r_y = center1.y;
                l_l = std::abs(rect0.br().y - rect0.tl().y);
                r_l = std::abs(rect1.br().y - rect1.tl().y);
                l_r_h = std::abs(l_x - r_x);
                l_r_v = std::abs(l_y - r_y);
            }
            else { // 0 => right
                l_x = center1.x;
                l_y = center1.y;
                r_x = center0.x;
                r_y = center0.y;
                l_l = std::abs(rect1.br().y - rect1.tl().y);
                r_l = std::abs(rect0.br().y - rect0.tl().y);
                l_r_h = std::abs(l_x - r_x);
                l_r_v = std::abs(l_y - r_y);
            }
            m_x = (l_x + r_x)/2;
            m_y = (l_y + r_y)/2 - 0.15*(l_l + r_l);
            m_l = 0.2*(l_l + r_l);
            l_m_h = l_r_h/2;
            m_r_h = l_r_h/2;
            l_m_v = 0.15*(l_l + r_l);
            m_r_v = 0.15*(l_l + r_l);
        }
        else { // left and middle or right and middle
            ROS_INFO ("left-middle or right-middle scenario");
            cv::Rect2d rect0; cv::Rect2d rect1;
            rect0 = cv::boundingRect (cv::Mat(contours[0]));
            rect1 = cv::boundingRect (cv::Mat(contours[1]));
            cv::Point2d center0; cv::Point2d center1;
            center0 = rect_center (rect0); center1 = rect_center (rect1);
            if (center0.x < center1.x) { // 0 => left, 1 => middle
                l_x = center0.x;
                l_y = center0.y;
                m_x = center1.x;
                m_y = center1.y;
                l_l = std::abs(rect0.br().y - rect0.tl().y);
                m_l = std::abs(rect1.br().y - rect1.tl().y);
                l_m_h = std::abs(l_x - m_x);
                l_m_v = std::abs(l_y - m_y);
                r_x = m_x + l_m_h;
                r_y = l_y;
                r_l = 0.9*l_l;
                m_r_h = l_m_h;
                m_r_v = l_m_v;
                l_r_h = 2*l_m_h;
                l_r_v = 0;
            }
            else { // 0 => right
                m_x = center1.x;
                m_y = center1.y;
                r_x = center0.x;
                r_y = center0.y;
                m_l = std::abs(rect1.br().y - rect1.tl().y);
                r_l = std::abs(rect0.br().y - rect0.tl().y);
                m_r_h = std::abs(m_x - r_x);
                m_r_v = std::abs(m_y - r_y);
                l_x = m_x - m_r_h;
                l_y = r_y;
                l_l = 0.9*r_l;
                l_m_h = m_r_h;
                l_r_h = 2*m_r_h;
                l_m_v = m_r_v;
                l_r_v = 0;
            }
        }
    }
    else {
        ROS_INFO ("3 contours");
        std::vector<cv::Rect2d> rects;
        std::vector<cv::Point2d> centers;
        for (int i = 0; i < 3; i++) {
            cv::Rect2d rect = cv::boundingRect(cv::Mat(contours[i]));
            rects.push_back(rect);
        }
        for (int i = 0; i < 3; i++) {
            centers.push_back (rect_center(rects[i]));
        }
        if (centers[0].x < centers[1].x) { // 0 => left, 1 => right
            l_x = centers[0].x;
            l_y = centers[0].y;
            r_x = centers[1].x;
            r_y = centers[1].y;
            m_x = centers[2].x;
            m_y = centers[2].y;
            l_l = std::abs(rects[0].br().y - rects[0].tl().y);
            r_l = std::abs(rects[1].br().y - rects[1].tl().y);
            m_l = std::abs(rects[2].br().y - rects[2].tl().y);
            l_r_h = std::abs(l_x - r_x);
            l_m_h = std::abs(l_x - m_x);
            m_r_h = std::abs(r_x - m_x);
            l_m_v = std::abs(l_y - m_y);
            m_r_v = std::abs(r_y - m_y);
            l_r_v = std::abs(l_y - r_y);
        }
        else { // 0 => right
            l_x = centers[1].x;
            l_y = centers[1].y;
            r_x = centers[0].x;
            r_y = centers[0].y;
            m_x = centers[2].x;
            m_y = centers[2].y;
            l_l = std::abs(rects[1].br().y - rects[1].tl().y);
            r_l = std::abs(rects[0].br().y - rects[0].tl().y);
            m_l = std::abs(rects[2].br().y - rects[2].tl().y);
            l_r_h = std::abs(l_x - r_x);
            l_m_h = std::abs(l_x - m_x);
            m_r_h = std::abs(r_x - m_x);
            l_m_v = std::abs(l_y - m_y);
            m_r_v = std::abs(r_y - m_y);
            l_r_v = std::abs(l_y - r_y);
        }
    }
    fill_msg (feature_msg, l_x, l_y, m_x, m_y, r_x, r_y, l_l, m_l, r_l, l_m_h, m_r_h, l_r_h, l_m_v, m_r_v, l_r_v);
    features_pub.publish(feature_msg);
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

            front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg();
            front_thresholded_pub.publish(front_image_thresholded_msg);
            front_roi_pub.publish(front_image_thresholded_msg);

            extractFeatures (image_front_thresholded);
            center = findGateCenter (image_front_thresholded);
            if (center == cv::Point(-1, -1)) { ROS_ERROR ("Gate center not found!!"); continue; }

            cv::circle(marked_img, center, 10, cv::Scalar(0, 250, 0), -1, 8, 1);
            // cv::circle(marked_img, cv::Point(temp_src.cols/2, temp_src.rows/2), 4, cv::Scalar(150, 150, 150), -1, 8, 0);

            front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg();
            front_marked_pub.publish(front_image_marked_msg);

            front_x_coordinate.data = 0;
            front_y_coordinate.data = center.x - temp_src.cols/2;
            front_z_coordinate.data = temp_src.rows/2 - center.y;

            std::cout << "Center of Gate: " <<  front_y_coordinate.data << ", " << front_z_coordinate.data << std::endl;

            front_x_coordinate_pub.publish(front_x_coordinate);
            front_y_coordinate_pub.publish(front_y_coordinate);
            front_z_coordinate_pub.publish(front_z_coordinate);
        }
        else ROS_INFO("Image empty");
        loop_rate.sleep();
        ros::spinOnce();
    }
}
