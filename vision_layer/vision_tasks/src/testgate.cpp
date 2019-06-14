#include <testgate.h>

TestGate::TestGate()
{
    loadParams();
    front_roi_pub = it.advertise("/anahita/roi", 1);
    image_rect_sub = it.subscribe("/anahita/left/image_rect_color", 1, &TestGate::rectCB, this);
    contour_center_client = nh.serviceClient<vision_tasks::ContourCenter>("contour_center");
    normal_server = nh.advertiseService("/anahita/target_normal", &TestGate::getNormal, this);
    ROS_INFO("Test gate node started"); 
}

bool TestGate::getNormal (master_layer::TargetNormal::Request &req,
                           master_layer::TargetNormal::Response &resp) {
    ROS_INFO ("Service called");
    ros::Rate loop_rate(20);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    
    while (ros::ok()) {
        if (close_task) {
            close_task = false;
            break;
        }
        if (!rect_thresholded.empty()) {
            cv::findContours (rect_thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
            contours = vision_commons::Contour::filterContours (contours, 100);
            if (contours.size() < 2) continue;
            vision_commons::Contour::sortFromBigToSmall (contours);
            std::vector<cv::Point> contour1 = contours[0];
            std::vector<cv::Point> contour2 = contours[1];
            cv::Rect rect1 = cv::boundingRect (cv::Mat(contour1));
            cv::Rect rect2 = cv::boundingRect (cv::Mat(contour2));
            vision_tasks::ContourCenter srv1;
            vision_tasks::ContourCenter srv2;

            srv1.request.br_x = rect1.br().x; 
            srv1.request.br_y = rect1.br().y;
            srv1.request.tl_x = rect1.tl().x;
            srv1.request.tl_y = rect1.tl().y;

            srv2.request.br_x = rect2.br().x; 
            srv2.request.br_y = rect2.br().y;
            srv2.request.tl_x = rect2.tl().x;
            srv2.request.tl_y = rect2.tl().y;

            contour_center_client.call(srv1);
            contour_center_client.call(srv2);

            cv::Point3d point1 (srv1.response.x, srv1.response.y, srv1.response.z);
            cv::Point3d point2 (srv2.response.x, srv2.response.y, srv2.response.z);

            ROS_INFO ("Point 1: %f %f %f", srv1.response.x, srv1.response.y, srv1.response.z);
            ROS_INFO ("Point 2: %f %f %f", srv2.response.x, srv2.response.y, srv2.response.z);
            resp.angle = atan((srv2.response.z - srv1.response.z)/(srv2.response.x - srv1.response.x));
            break;
        }
        else ROS_INFO("Image empty");
        loop_rate.sleep();
    }
    return true;
}

void TestGate::rectCB (const sensor_msgs::Image::ConstPtr &msg) {
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

void TestGate::loadParams()
{
    nh.getParam("/anahita/vision/startgate/b_min", front_low_b_);
    nh.getParam("/anahita/vision/startgate/b_max", front_high_b_);
    nh.getParam("/anahita/vision/startgate/g_min", front_low_g_);
    nh.getParam("/anahita/vision/startgate/g_max", front_high_g_);
    nh.getParam("/anahita/vision/startgate/r_min", front_low_r_);
    nh.getParam("/anahita/vision/startgate/r_max", front_high_r_);
    nh.getParam("/anahita/vision/startgate/closing_mat_point", front_closing_mat_point_);
    nh.getParam("/anahita/vision/startgate/closing_iter", front_closing_iter_);
    nh.getParam("/anahita/vision/startgate/opening_mat_point", front_opening_mat_point_);
    nh.getParam("/anahita/vision/startgate/opening_iter", front_opening_iter_);
    nh.getParam("/anahita/vision/startgate/bilateral_iter", front_bilateral_iter_);
}

void TestGate::spinThreadFront() {
    cv::Mat temp_src;
    std::vector<cv::Point> largest_contour;
    ros::Rate loop_rate(15);

    while (ros::ok()) {
        if (close_task) {
            close_task = false;
            break;
        }
        if (!image_front.empty()) {
            temp_src = image_front.clone();
        
            vision_commons::Filter::bilateral(temp_src, front_bilateral_iter_);
            image_front_thresholded = vision_commons::Threshold::threshold(temp_src, front_low_b_, front_high_b_,
                                                                           front_low_g_, front_high_g_, front_low_r_, 
                                                                           front_high_r_);
            if (!rect_image.empty()) {
                rect_thresholded = vision_commons::Threshold::threshold(rect_image, front_low_b_, front_high_b_,
                                                                        front_low_g_, front_high_g_, front_low_r_, 
                                                                        front_high_r_);
                front_roi_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", rect_thresholded).toImageMsg());
            }
            vision_commons::Morph::open(image_front_thresholded, 2 * front_opening_mat_point_ + 1,
                                        front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
            vision_commons::Morph::close(image_front_thresholded, 2 * front_closing_mat_point_ + 1,
                                         front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);
            largest_contour = vision_commons::Contour::getLargestContour(image_front_thresholded);
            if (!largest_contour.size()) {
                ROS_INFO("No contour found");
                continue;
            }

            front_marked_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg());
            front_thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg());
            
        }
        else {
            ROS_INFO("Image empty");
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}
