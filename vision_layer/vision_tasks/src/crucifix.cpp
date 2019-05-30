#include <crucifix.h>

Crucifix::Crucifix() {
	this->loadParams ();
	this->front_roi_pub = it.advertise("/anahita/roi", 1);
}

Crucifix::~Crucifix() {}

void Crucifix::loadParams () {
	nh.getParam("/anahita/vision/crucifix/b_min", front_low_b_);
	nh.getParam("/anahita/vision/crucifix/b_max", front_high_b_);
	nh.getParam("/anahita/vision/crucifix/g_min", front_low_g_);
	nh.getParam("/anahita/vision/crucifix/g_max", front_high_g_);
	nh.getParam("/anahita/vision/crucifix/r_min", front_low_r_);
	nh.getParam("/anahita/vision/crucifix/r_max", front_high_r_);
	nh.getParam("/anahita/vision/crucifix/closing_mat_point", front_closing_mat_point_);
	nh.getParam("/anahita/vision/crucifix/closing_iter", front_closing_iter_);
	nh.getParam("/anahita/vision/crucifix/opening_mat_point", front_opening_mat_point_);
	nh.getParam("/anahita/vision/crucifix/opening_iter", front_opening_iter_);
	nh.getParam("/anahita/vision/crucifix/bilateral_iter", front_bilateral_iter_);
}

void Crucifix::spinThreadFront()
{
	cv::Mat image_marked;
	std::vector<cv::Point> largest_contour;
	cv::Rect bound_rect;
    cv::Scalar line_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar edge_color(255, 255, 255);
	cv::Scalar hough_line_color(0, 164, 253);
	cv::Scalar contour_color(255, 0, 0);
	cv::Scalar bound_rect_color(255, 255, 255);
	cv::Point bound_rect_center;
   	std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> lines;   
	sensor_msgs::ImagePtr front_image_marked_msg;
	sensor_msgs::ImagePtr front_image_thresholded_msg;
	ros::Rate loop_rate(15);
    ROS_INFO("__________________________________________________");
    cv::VideoCapture cap(1);
    if(!cap.isOpened())  // check if we succeeded
        ROS_INFO("print");
	while (ros::ok())
	{
        cap >> image_front;
		if (close_task) {
			close_task = false;
			break;
		}
		if (!image_front.empty()) {
			ROS_INFO("Found Image");
			image_marked = image_front.clone();
            cv::cvtColor(image_front, image_front, CV_BGR2HSV);
			vision_commons::Filter::bilateral(image_front, front_bilateral_iter_);

			image_front_thresholded = vision_commons::Threshold::threshold(image_front, front_low_b_, front_high_b_, front_low_g_, front_high_g_, front_low_r_, front_high_r_);
			vision_commons::Morph::open(image_front_thresholded, 2 * front_opening_mat_point_ + 1, front_opening_mat_point_, front_opening_mat_point_, front_opening_iter_);
			vision_commons::Morph::close(image_front_thresholded, 2 * front_closing_mat_point_ + 1, front_closing_mat_point_, front_closing_mat_point_, front_closing_iter_);

			cv::Mat edges(image_front_thresholded.rows, image_front_thresholded.cols, CV_8UC1, cv::Scalar::all(0));
    		contours = vision_commons::Contour::getBestX(image_front_thresholded, 2);
    		cv::drawContours(edges, contours, 0, edge_color, 1, 8);

            if(contours.size()!= 0)
            {
                ROS_INFO("Contour size");
                cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 60, 70, 10);
   				std::vector<double> angles;
				ROS_INFO("Lines size: %d", lines.size());
                for (int i = 0; i < lines.size(); i++)
				{
					if ((lines[i][2] == lines[i][0]) || (lines[i][1] == lines[i][3]))
						continue;
			    	angles.push_back(atan(static_cast<double>(lines[i][2] - lines[i][0]) / (lines[i][1] - lines[i][3])) * 180.0 / 3.14159);
					cv::line(image_marked, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), hough_line_color, 1, CV_AA);
				}
                for(int i = 0; i<angles.size(); i++)
                    ROS_INFO("Conventional printing: %f", angles[i]);
            }           
            ROS_INFO("Contour end");

            front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_front).toImageMsg();
            front_marked_pub.publish(front_image_marked_msg);

            front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_front_thresholded).toImageMsg();
            front_thresholded_pub.publish(front_image_thresholded_msg);

			cv::namedWindow("EDGES");
			cv::imshow( "EDGES", edges);
	                   // Show our image inside it.
		}
		else {
			ROS_INFO("Image empty");
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}

void Crucifix::spinThreadBottom () {}
