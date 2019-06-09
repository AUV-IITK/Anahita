#include <testgate.h>

StartGate::StartGate()
{
    loadParams();
    front_roi_pub = it.advertise("/anahita/roi", 1);
    image_rect_sub = it.subscribe("/anahita/left/image_rect_color", 1, &StartGate::rectCB, this);
    contour_center_client = nh.serviceClient<vision_tasks::ContourCenter>("contour_center");
}

void StartGate::rectCB (const sensor_msgs::Image::ConstPtr &msg) {
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

void StartGate::loadParams()
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

void StartGate::spinThreadFront()
{
    cv::Mat temp_src;
    std::vector<cv::Point> largest_contour;
    ros::Rate loop_rate(15);
    cv::Mat rect_thresholded;

    while (ros::ok())
    {
        if (close_task)
        {
            close_task = false;
            break;
        }
        if (!image_front.empty())
        {
            temp_src = image_front.clone();
        
            vision_commons::Filter::bilateral(temp_src, front_bilateral_iter_);
            image_front_thresholded = vision_commons::Threshold::threshold(temp_src, front_low_b_, front_high_b_,
                                                                           front_low_g_, front_high_g_,
                                                                           front_low_r_, front_high_r_);
            if (!rect_image.empty()) {
                rect_thresholded = vision_commons::Threshold::threshold(rect_image, front_low_b_, front_high_b_,
                                                                            front_low_g_, front_high_g_,
                                                                            front_low_r_, front_high_r_);
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
        else
        {
            ROS_INFO("Image empty");
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}
