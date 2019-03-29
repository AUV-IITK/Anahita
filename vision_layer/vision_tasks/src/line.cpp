#include <line.h>


Line::Line(){
	this->bottom_low_h_ = 31;
	this->bottom_high_h_ = 47;
	this->bottom_low_s_ = 0;
	this->bottom_high_s_ = 255;
	this->bottom_low_v_ = 0;
	this->bottom_high_v_ = 255;
	this->bottom_opening_mat_point_ = 1;
	this->bottom_opening_iter_ = 0;
	this->bottom_closing_mat_point_ = 2;
	this->bottom_closing_iter_ = 1;

	this->camera_frame_ = "auv-iitk";
	image_transport::ImageTransport it(nh);
	this->detection_pub = nh.advertise<std_msgs::Bool>("/detected", 1000);
	this->bottom_coordinates_pub = nh.advertise<geometry_msgs::Pose2D>("/line_task/line_coordinates", 1000);
	this->image_raw_sub = it.subscribe("/anahita/bottom_camera/image_raw", 1, &Line::imageCallback, this);
}

void Line::callback(vision_tasks::lineRangeConfig &config, double level)
{
	Line::bottom_low_h_ = config.low_h;
	Line::bottom_low_s_ = config.low_s;
	Line::bottom_low_v_ = config.low_v;
	Line::bottom_high_h_ = config.high_h;
	Line::bottom_high_s_ = config.high_s;
	Line::bottom_high_v_ = config.high_v;
	Line::bottom_opening_mat_point_ = config.opening_mat_point;
	Line::bottom_opening_iter_ = config.opening_iter;
	Line::bottom_closing_mat_point_ = config.closing_mat_point;
	Line::bottom_closing_iter_ = config.closing_iter;
};

double Line::computeMean(std::vector<double> &newAngles)
{
	double sum = 0;
	for(int  i =0; i < newAngles.size(); i++)
		sum += newAngles[i];
	return (double) sum/newAngles.size();
}

void Line::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	try
	{
		image_bottom = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (cv::Exception &e)
	{
		ROS_ERROR("cv exception: %s", e.what());
	}
}

void Line::spinThreadBottom() {
	dynamic_reconfigure::Server<vision_tasks::lineRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::lineRangeConfig>::CallbackType f;
	f = boost::bind(&Line::callback, this, _1, _2);
	server.setCallback(f);

	cv::Scalar line_center_color(255, 255, 255);
	cv::Scalar image_center_color(0, 0, 0);
	cv::Scalar edge_color(255, 255, 255);
	cv::Scalar hough_line_color(0, 164, 253);
	cv::Scalar contour_color(255, 0, 0);

	cv::Mat image_hsv;
	cv::Mat image_thresholded;
  	std::vector<std::vector<cv::Point> > contours;
	cv::RotatedRect bounding_rectangle;
	std::vector<cv::Vec4i> lines;
	geometry_msgs::Pose2D line_point_message;
	cv::Mat image_marked;

	std_msgs::Bool detection_bool;

	while (ros::ok())
	{
		if (task_done) {
			break;
		}
		if (!image_bottom.empty())
		{
			image_bottom.copyTo(image_marked);
			if (bottom_high_h_ > bottom_low_h_ && bottom_high_s_ > bottom_low_s_ && bottom_high_v_ > bottom_low_v_)
			{
				cv::cvtColor(image_bottom, image_hsv, CV_BGR2HSV);
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, bottom_low_h_, bottom_high_h_, bottom_low_s_, bottom_high_s_, bottom_low_v_, bottom_high_v_);
				image_thresholded = vision_commons::Morph::open(image_thresholded, 2 * bottom_opening_mat_point_ + 1, bottom_opening_mat_point_, bottom_opening_mat_point_, bottom_opening_iter_);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 2 * bottom_closing_mat_point_ + 1, bottom_closing_mat_point_, bottom_closing_mat_point_, bottom_closing_iter_);
				contours = vision_commons::Contour::getBestX(image_thresholded, 1);
				cv::Mat edges(image_thresholded.rows, image_thresholded.cols, CV_8UC1, cv::Scalar::all(0));
				std::vector<double> angles;

				cv::drawContours(edges, contours, 0, edge_color, 1, 8);
				if (contours.size() != 0)
				{
					bounding_rectangle = cv::minAreaRect(cv::Mat(contours[0]));
					cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 60, 70, 10);
					for (int i = 0; i < lines.size(); i++)
					{
						if ((lines[i][2] == lines[i][0]) || (lines[i][1] == lines[i][3]))
							continue;
						angles.push_back(atan(static_cast<double>(lines[i][2] - lines[i][0]) / (lines[i][1] - lines[i][3])) * 180.0 / 3.14159);
						cv::line(image_marked, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), hough_line_color, 1, CV_AA);
					}
					bottom_y_coordinate.data = (image_bottom.size().height) / 2 - bounding_rectangle.center.y;
					bottom_x_coordinate.data = bounding_rectangle.center.x - (image_bottom.size().width) / 2;
					if (angles.size() > 0)
					{
						double angle = computeMean(angles);
						if (angle > 90.0)
							bottom_z_coordinate.data = angle - 90.0;
						else
							bottom_z_coordinate.data = angle;
					}
					else
						bottom_z_coordinate.data = 0.0;

					int non_zero = countNonZero(image_thresholded);

					if (non_zero > (3072 * 0.20))
						detection_bool.data=true;
					else
						detection_bool.data=false;						
					
					ROS_INFO("Line (x, y, theta) = (%.2f, %.2f, %.2f)", bottom_y_coordinate.data, bottom_x_coordinate.data, bottom_z_coordinate.data);
					cv::circle(image_marked, cv::Point(bounding_rectangle.center.x, bounding_rectangle.center.y), 1, line_center_color, 8, 0);
					cv::circle(image_marked, cv::Point(image_bottom.size().width / 2, image_bottom.size().height / 2), 1, image_center_color, 8, 0);

					for (int i = 0; i < contours.size(); i++)
					{
						cv::drawContours(image_marked, contours, i, contour_color, 1, 8);
					}
				}
			}
			bottom_x_coordinates_pub.publish(bottom_y_coordinate);
			bottom_y_coordinates_pub.publish(bottom_x_coordinate);
			bottom_z_coordinates_pub.publish(bottom_z_coordinate);
			detection_pub.publish(detection_bool);
			bottom_thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg());
			bottom_coordinates_pub.publish(line_point_message);
			bottom_marked_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_marked).toImageMsg());
		}
		else
			ROS_INFO("Image empty!");
		ros::spinOnce();
	}
}