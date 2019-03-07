#include <base_class.h>


//add canny
//bottom/front_coordinates_pub
Base_class::Base_class(){
	image_transport::ImageTransport it(nh);

	this->front_blue_filtered_pub = it.advertise("/task/blue_filtered", 1);
	this->front_thresholded_pub = it.advertise("/task/thresholded", 1);
	this->front_marked_pub = it.advertise("/task/marked", 1);
	this->bottom_blue_filtered_pub = it.advertise("/task/bottom/blue_filtered", 1);
    this->bottom_thresholded_pub = it.advertise("/task/bottom/thresholded", 1);
    this->bottom_marked_pub = it.advertise("/task/bottom/marked", 1);

	this->x_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/x_coordinate", 1000);
	this->y_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/y_coordinate", 1000);
	this->z_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/z_coordinate", 1000);

	this->task_done_pub = nh.advertise<std_msgs::Bool>("/task/done", 1000);
	this->detection_pub = nh.advertise<std_msgs::Bool>("/detected", 1000);
}

void Base_class::frontTaskHandling (bool status) {
	if(status)
	{
		spin_thread = new boost::thread(boost::bind(&Base_class::spinThread, this)); 
	}
	else 
	{
		close_task = true;
        spin_thread->join();
		std::cout << "Front Task Handling function over" << std::endl;	
	}
}

void Base_class::bottomTaskHandling(bool status) {
	if(status)
	{
		spin_thread_bottom = new boost::thread(&Base_class::spinThreadBottom, this); 
	}
	else 
	{
		task_done = true;
        spin_thread_bottom->join();
		std::cout << "Bottom Task Handling function over" << std::endl;	
	}
}

// void Base_class::imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg)
// {
// 	cv_bridge::CvImagePtr cv_img_ptr;
// 	try
// 	{
// 		image_front = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
// 		// ROS_INFO("Found a new image and stored it in image_front!");
// 	}
// 	catch (cv_bridge::Exception &e)
// 	{
// 		ROS_ERROR("cv_bridge exception: %s", e.what());
// 	}
// 	catch (cv::Exception &e)
// 	{
// 		ROS_ERROR("cv exception: %s", e.what());
// 	}
// };

// void Base_class::imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg)
// {
// 	cv_bridge::CvImagePtr cv_img_ptr;
// 	try
// 	{
// 		image_bottom = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
// 	}
// 	catch (cv_bridge::Exception &e)
// 	{
// 		ROS_ERROR("cv_bridge exception: %s", e.what());
// 	}
// 	catch (cv::Exception &e)
// 	{
// 		ROS_ERROR("cv exception: %s", e.what());
// 	}
// };

// void Base_class::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
// {
// 	try
// 	{
// 		image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
// 	}
// 	catch (cv_bridge::Exception &e)
// 	{
// 		ROS_ERROR("cv_bridge exception: %s", e.what());
// 	}
// 	catch (cv::Exception &e)
// 	{
// 		ROS_ERROR("cv exception: %s", e.what());
// 	}
// }

// void Base_class::switchColor(int color)
// {
// 	if(color > 1)
// 		std::cerr << "Changing to wrong torpedo/buoy color, use 0-1 for the the different colors" << std::endl;
// 	else
// 	{
// 		current_color = color;		
// 		this->front_low_h_ = this->data_low_h[current_color];
// 		this->front_high_h_ = this->data_high_h[current_color];
// 		this->front_low_s_ = this->data_low_s[current_color];
// 		this->front_high_s_ = this->data_high_s[current_color];
// 		this->front_low_v_ = this->data_low_v[current_color];
// 		this->front_high_v_ = this->data_high_v[current_color];
// 	}
// 	std::cout << "Colour changed successfully" << std::endl;
// }
