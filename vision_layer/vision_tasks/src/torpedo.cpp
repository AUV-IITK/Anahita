#include <torpedo.h>

Torpedo::Torpedo(){
	this->loadParams ();
    this->front_roi_pub = it.advertise("/anahita/roi", 1);
    service = nh.advertiseService("/anahita/change_torpedo_hole", &Torpedo::changeTorpedoHole, this);

    std::string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    std::string trackerType = trackerTypes[1];
 
    if (trackerType == "KCF") {
        tracker1 = cv::TrackerKCF::create();
        tracker2 = cv::TrackerKCF::create();
        tracker3 = cv::TrackerKCF::create();
    }
    else if (trackerType == "MIL") {
        tracker1 = cv::TrackerMIL::create();
        tracker2 = cv::TrackerMIL::create();
        tracker3 = cv::TrackerMIL::create();
    }
}

bool Torpedo::changeTorpedoHole (master_layer::ChangeTorpedoHole::Request &req,
                        master_layer::ChangeTorpedoHole::Response &resp) {
    current_hole = req.hole;
    ROS_INFO ("Torpedo target changed to: %s", current_hole);
    return true;
}

void Torpedo::loadParams(){
	nh.getParam("/anahita/vision/torpedo_holes/b_min", front_low_b_);
	nh.getParam("/anahita/vision/torpedo_holes/b_max", front_high_b_);
	nh.getParam("/anahita/vision/torpedo_holes/g_min", front_low_g_);
	nh.getParam("/anahita/vision/torpedo_holes/g_max", front_high_g_);
	nh.getParam("/anahita/vision/torpedo_holes/r_min", front_low_r_);
	nh.getParam("/anahita/vision/torpedo_holes/r_max", front_high_r_);
	nh.getParam("/anahita/vision/torpedo_holes/closing_mat_point", front_closing_mat_point_);
	nh.getParam("/anahita/vision/torpedo_holes/closing_iter", front_closing_iter_);
	nh.getParam("/anahita/vision/torpedo_holes/opening_mat_point", front_opening_mat_point_);
	nh.getParam("/anahita/vision/torpedo_holes/opening_iter", front_opening_iter_);
	nh.getParam("/anahita/vision/torpedo_holes/bilateral_iter", front_bilateral_iter_);
}

void Torpedo::recogniseHoles (cv::Mat& thres_img) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours (thres_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::vector<int> idx;
    float area = 0;

    for (int i = 0; i < contours.size(); i++) {
        area = cv::contourArea (contours[i], false);
        if (area > 200) {
            if (hierarchy[i][2] < 0 && hierarchy[i][3] >= 0) continue;
            idx.push_back (i);
        }
    }
    
    ROS_INFO ("No. of contours: %d", idx.size());
    if (idx.size() > 3) ROS_ERROR ("No. of contours greater than 3");
    if (idx.size() == 0) return;

    std::vector<cv::Point> points;
    cv::Point2f center_;
    float radius_;

    for (int i = 0; i < idx.size(); i++) {
        cv::minEnclosingCircle (cv::Mat(contours[idx[i]]), center_, radius_);
        cv::circle(marked_img, center_, (int)radius_, cv::Scalar(0, 0, 255), 2, 8, 0);
        center_.x = center_.x - thres_img.cols/2;
        center_.y = thres_img.rows/2 - center_.y;
        points.push_back(center_);
    }

    updateCoordinates (points);
}

bool x_cmp (cv::Point a, cv::Point b) {
    return a.x < b.x;
}

bool y_cmp (cv::Point a, cv::Point b) {
    return a.y < b.y;
}

void Torpedo::updateCoordinates (std::vector<cv::Point> points) {
    if (points.size() == 3) {
        std::sort (points.begin(), points.end(), x_cmp);
        TR.x = points[2].x;
        TR.y = points[2].y;
        TL.x = points[0].x;
        TL.y = points[0].y;
        BOT.x = points[1].x;
        BOT.y = points[1].y;

        TL_init = true;
        TR_init = true;
        BOT_init = true;
    }
    else if (points.size() == 2) {
        std::sort (points.begin(), points.end(), x_cmp);
        if (points[0].y < points[1].y + 50) {
            TR.x = points[1].x;
            TR.y = points[1].y;
            BOT.x = points[0].x;
            BOT.y = points[0].y;
            TL_init = false;
            TR_init = true;
            BOT_init = true;
        }
        else if (points[1].y < points[0].y + 50) {
            TL.x = points[0].x;
            TL.y = points[0].y;
            BOT.x = points[1].x;
            BOT.y = points[1].y;
            TL_init = true;
            TR_init = false;
            BOT_init = true;
        }
        else {
            TL.x = points[0].x;
            TL.y = points[0].y;
            TR.x = points[1].x;
            TR.y = points[1].y;
            TL_init = true;
            TR_init = true;
            BOT_init = false;
        }
    }
    else {
        TR.x = points[0].x;
        TR.y = points[0].y;
        TL.x = points[0].x;
        TL.y = points[0].y;
        BOT.x = points[0].x;
        BOT.y = points[0].y;
        TL_init = true;
        TR_init = true;
        BOT_init = true;
    }
}

cv::Point2f Torpedo::threshROI (const cv::Rect2d& bounding_rect, const cv::Mat& img, int padding) {
    cv::Rect2d roi (bounding_rect.x, bounding_rect.y, bounding_rect.width + 2*padding, bounding_rect.height + 2*padding);
    cv::Mat roi_img = img(roi);
    cv::Mat thresholded;
    thresholded = vision_commons::Threshold::threshold(roi_img, front_low_b_, front_high_b_,
                                                       front_low_g_, front_high_g_,
                                                       front_low_r_, front_high_r_);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours (thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    float area = 0;
    cv::Point2f center_;
    float radius_;
    
    for (int i = 0; i < contours.size(); i++) {
        area = cv::contourArea (contours[i], false);
        if (area > 0.2*roi_img.rows*roi_img.cols) {
            if (hierarchy[i][2] < 0 && hierarchy[i][3] > 0) continue;
            cv::minEnclosingCircle (cv::Mat(contours[i]), center_, radius_);
            center_.x = center_.x - roi_img.cols/2 + bounding_rect.x;
            center_.y = roi_img.rows/2 - center_.y + bounding_rect.y;
            return center_;
        } 
    }
}

void Torpedo::findCircles (cv::Mat& src_img, cv::Mat& thres_img, double circle_threshold) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours (thres_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    
    if (!initTracker && contours.size() >= 3) {
        InitTracker(src_img, thres_img, circle_threshold);
    }

    float area = 0;
    cv::Point2f center_;
    float radius_;
    int circle_cnt = 0;

    ROS_INFO ("contour size: %d", contours.size());

    for (int i = 0; i < contours.size(); i++) {
        area = cv::contourArea (contours[i], false);
        if (area > circle_threshold) {
            circle_cnt++;
            if (hierarchy[i][2] < 0 && hierarchy[i][3] > 0) continue;
            cv::minEnclosingCircle (cv::Mat(contours[i]), center_, radius_);
            cv::circle(marked_img, center_, (int)radius_, cv::Scalar(0, 0, 255), 2, 8, 0);
        }
    }
    ROS_INFO ("No. of contours: %d", circle_cnt);
}

void Torpedo::InitTracker (cv::Mat& src_img, cv::Mat& thres_img, double circle_threshold) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours (thres_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::vector<int> idx;
    float area = 0;

    for (int i = 0; i < contours.size(); i++) {
        area = cv::contourArea (contours[i], false);
        if (area > circle_threshold) {
            if (hierarchy[i][2] < 0 && hierarchy[i][3] > 0) continue;
            idx.push_back (i);
        }
    }
    if (idx.size() != 3) return;

    bbox1 = boundingRect(cv::Mat(contours[idx[0]]));
    bbox2 = boundingRect(cv::Mat(contours[idx[1]]));
    bbox3 = boundingRect(cv::Mat(contours[idx[2]]));

    rectangle(marked_img, bbox1, cv::Scalar(255, 0, 0), 2, 1);
    rectangle(marked_img, bbox2, cv::Scalar(0, 255, 0), 2, 1);
    rectangle(marked_img, bbox3, cv::Scalar(0, 0, 255), 2, 1);

    tracker1->init(src_img, bbox1);
    tracker2->init(src_img, bbox2);
    tracker3->init(src_img, bbox3);
    initTracker = true;
}

void Torpedo::updateTracker (cv::Mat& src_img) {
    if (tracker1->update (src_img, bbox1))
        rectangle(marked_img, bbox1, cv::Scalar(255, 0, 0), 2, 1);
    else 
        ROS_ERROR ("BBox1 failing....");
    if (tracker2->update (src_img, bbox2))
        rectangle(marked_img, bbox2, cv::Scalar(0, 255, 0), 2, 1);
    else 
        ROS_ERROR ("BBox2 failing....");
    if (tracker3->update (src_img, bbox3))
        rectangle(marked_img, bbox3, cv::Scalar(0, 0, 255), 2, 1);
    else 
        ROS_ERROR ("BBox3 failing....");

}

void Torpedo::spinThreadFront() {
    cv::Mat temp_src;
    std::vector<cv::Point> largest_contour;
    cv::Rect bound_rect;
    cv::Scalar bound_rect_color(255, 255, 255);
    cv::Point bound_rect_center;
    cv::Point bound_circle_center;
    sensor_msgs::ImagePtr front_image_marked_msg;
    sensor_msgs::ImagePtr front_image_thresholded_msg;
    ros::Rate loop_rate(15);
    std::vector<cv::Vec3f> circles;
                ROS_INFO("adsdasd");

    cv::Mat roi;
    ROS_INFO("Haha");

	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_front.empty())
		{
            ROS_INFO("adsdasd");

            vision_mutex.lock();
            temp_src = image_front.clone();
            marked_img = image_front.clone();
            vision_mutex.unlock();
	    ROS_INFO("LOL");
            vision_commons::Filter::bilateral(temp_src, front_bilateral_iter_);
            image_front_thresholded = vision_commons::Threshold::threshold(temp_src, front_low_b_, front_high_b_,
                                                                           front_low_g_, front_high_g_,
                                                                           front_low_r_, front_high_r_);
            vision_commons::Morph::open(image_front_thresholded, 2 * front_opening_mat_point_ + 1,
                                        front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
            vision_commons::Morph::close(image_front_thresholded, 2 * front_closing_mat_point_ + 1,
                                         front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);
            largest_contour = vision_commons::Contour::getLargestContour(image_front_thresholded);
            if (largest_contour.size()) {
                ROS_INFO(" contour found");

	    // findCircles (temp_src, image_front_thresholded, 300);
            // updateTracker (temp_src);
            recogniseHoles (image_front_thresholded);

            if (!(TL_init||BOT_init||TR_init)) {
                ROS_INFO ("Not visible hole");
                continue;
            }

            if (current_hole == "") continue;
            else if (current_hole == "TR") {bound_circle_center.x = TR.x; bound_circle_center.y = TR.y;}
            else if (current_hole == "TL") {bound_circle_center.x = TL.x; bound_circle_center.y = TL.y;}
            else if (current_hole == "BOT") {bound_circle_center.x = BOT.x; bound_circle_center.y = BOT.y;}

            }
		
            
            front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", marked_img).toImageMsg();
            front_marked_pub.publish(front_image_marked_msg);

            front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg();
            front_thresholded_pub.publish(front_image_thresholded_msg);
            front_roi_pub.publish(front_image_thresholded_msg);

            front_x_coordinate.data = 0;
            front_y_coordinate.data = bound_circle_center.x;
            front_z_coordinate.data = bound_circle_center.y;

            std::cout << "Center of bound_circle_center: " <<  front_y_coordinate.data << ", " << front_z_coordinate.data << std::endl;

            front_x_coordinate_pub.publish(front_x_coordinate);
            front_y_coordinate_pub.publish(front_y_coordinate);
            front_z_coordinate_pub.publish(front_z_coordinate);
		}
		else
			ROS_INFO("Image empty");
		loop_rate.sleep();
		ros::spinOnce();
	}
}
