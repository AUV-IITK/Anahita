#include <triangular_buoy.h>

TriangularBuoy::TriangularBuoy () {
    loadParams ();
    roi_pub = it.advertise("/anahita/roi", 1);
    depth_sub = nh.subscribe("/anahita/mean_coord", 100, &TriangularBuoy::depthCallback, this);
    depth_service = nh.advertiseService("/anahita/get_max_depth", &TriangularBuoy::depthRequest, this);
    blackout_service = nh.advertiseService("/anahita/get_blackout_time", &TriangularBuoy::blackoutCB, this);
    image_rect_sub = it.subscribe("/anahita/left/image_rect_color", 1, &TriangularBuoy::rectCB, this);

    ROS_INFO ("Triangular Buoy Vision node initialise");
}

void TriangularBuoy::depthCallback (const geometry_msgs::Point msg) {
    if (msg.z > max_depth) max_depth = msg.z;
}

bool TriangularBuoy::depthRequest (master_layer::GetMaxDepth::Request& req,
                                   master_layer::GetMaxDepth::Response& res) {
    res.depth = max_depth;
    return true;
}

void TriangularBuoy::rectCB (const sensor_msgs::Image::ConstPtr &msg) {
	try {
		rect_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image; 
    }
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what()); 
    }
	catch (cv::Exception &e) {
		ROS_ERROR("cv exception: %s", e.what()); 
    }
}

void TriangularBuoy::loadParams () {
    nh.getParam("/anahita/vision/triangular_buoy/b_min", bgr_min[0]);
    nh.getParam("/anahita/vision/triangular_buoy/b_max", bgr_max[0]);
    nh.getParam("/anahita/vision/triangular_buoy/g_min", bgr_min[1]);
    nh.getParam("/anahita/vision/triangular_buoy/g_max", bgr_max[1]);
    nh.getParam("/anahita/vision/triangular_buoy/r_min", bgr_min[2]);
    nh.getParam("/anahita/vision/triangular_buoy/r_max", bgr_max[2]);
    nh.getParam("/anahita/vision/triangular_buoy/closing_mat_point", front_closing_mat_point_);
    nh.getParam("/anahita/vision/triangular_buoy/closing_iter", front_closing_iter_);
    nh.getParam("/anahita/vision/triangular_buoy/opening_mat_point", front_opening_mat_point_);
    nh.getParam("/anahita/vision/triangular_buoy/opening_iter", front_opening_iter_);
    nh.getParam("/anahita/vision/triangular_buoy/bilateral_iter", front_bilateral_iter_);
}

void TriangularBuoy::preProcess (cv::Mat& temp_src) {
    vision_commons::Filter::bilateral(temp_src, front_bilateral_iter_);
    image_front_thresholded = vision_commons::Threshold::threshold(temp_src, bgr_min, bgr_max);
    vision_commons::Morph::open(image_front_thresholded, 2 * front_opening_mat_point_ + 1, 
                                front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
    vision_commons::Morph::close(image_front_thresholded, 2 * front_closing_mat_point_ + 1, 
                                front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);
}

cv::Point TriangularBuoy::findCenter () {
    static std::vector<std::vector<cv::Point> > contours;
    static std::vector<cv::Vec4i> hierarchy;
    
    cv::findContours (image_front_thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    contours = vision_commons::Contour::filterContours (contours, 100);
    
    if (!blackout_response_ready) { // block to calculate blackout time 
        if (!init && contours.size() != 0) {
            contour_init = true;
            init = true;
        }
        else if (contour_init && contours.size() == 0) {
            init_time = ros::Time::now().toSec();
            ros::Duration(0.1).sleep();
            contour_init = false;
        }
        else if (contours.size() != 0) {
            blackout_response_ready = true;
            blackout_time = ros::Time::now().toSec() - init_time;
        }
    }

    if (!contours.size()) return cv::Point(-1, -1);
    vision_commons::Contour::sortFromBigToSmall (contours);
    
    cv::Rect bound_rect = cv::boundingRect (cv::Mat(contours[0]));
    return cv::Point (((bound_rect.br()).x + (bound_rect.tl()).x)/2,
                        ((bound_rect.tl()).y + (bound_rect.br()).y)/2);
}

bool TriangularBuoy::blackoutCB (master_layer::GetBlackoutTime::Request& req,
                                 master_layer::GetBlackoutTime::Response& res) {
    res.blackout_time = blackout_time;
    return true;
}

void TriangularBuoy::spinThreadFront () {
	cv::Mat temp_src;
    ros::Rate loop_rate(20);
    cv::Point center;

	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_front.empty()) {
            vision_mutex.lock();
            image_front.copyTo(temp_src);
            vision_mutex.unlock();
            if (!rect_image.empty()) {
                rect_thresholded = vision_commons::Threshold::threshold(rect_image, bgr_min, bgr_max);
                roi_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", rect_thresholded).toImageMsg());
            }
            preProcess (temp_src);

            front_thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg());

            center = findCenter ();

            if (center == cv::Point(-1, -1)) continue;

            front_marked_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg());

            front_x_coordinate.data = 0;
            front_y_coordinate.data = center.x - temp_src.cols/2;
            front_z_coordinate.data = temp_src.rows/2 - center.y;

            front_x_coordinate_pub.publish(front_x_coordinate);
            front_y_coordinate_pub.publish(front_y_coordinate);
            front_z_coordinate_pub.publish(front_z_coordinate);

            std::cout << "Center of Triangular Buoy: " <<  front_y_coordinate.data << ", " << front_z_coordinate.data << std::endl;
		}
		else ROS_INFO("Image empty");
		loop_rate.sleep();
		ros::spinOnce();
	}
}
