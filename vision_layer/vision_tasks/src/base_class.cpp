#include <base_class.h>

Base_class::Base_class() : it(nh) {
	init();
}

void Base_class::init(){
	
	this->front_thresholded_pub = it.advertise("/anahita/front_camera/thresholded", 1);
	this->front_edges_pub = it.advertise("/anahita/front_camera/edges", 1);

	this->front_marked_pub = it.advertise("/anahita/front_camera/marked", 1);

    this->bottom_thresholded_pub = it.advertise("/anahita/bottom_camera/thresholded", 1);
    this->bottom_marked_pub = it.advertise("/anahita/bottom_camera/marked", 1);

	this->front_x_coordinate_pub = nh.advertise<std_msgs::Float32>("/anahita/front_camera/x_coordinate", 1);
	this->front_y_coordinate_pub = nh.advertise<std_msgs::Float32>("/anahita/front_camera/y_coordinate", 1);
	this->front_z_coordinate_pub = nh.advertise<std_msgs::Float32>("/anahita/front_camera/z_coordinate", 1);
	this->bottom_x_coordinate_pub = nh.advertise<std_msgs::Float32>("/anahita/bottom_camera/x_coordinate", 1);
	this->bottom_y_coordinate_pub = nh.advertise<std_msgs::Float32>("/anahita/bottom_camera/y_coordinate", 1);
	this->bottom_z_coordinate_pub = nh.advertise<std_msgs::Float32>("/anahita/bottom_camera/z_coordinate", 1);

	this->detection_pub = nh.advertise<std_msgs::Bool>("/detected", 1);
	this->front_image_sub = it.subscribe("/anahita/front_camera/image_raw", 1, &Base_class::imageFrontCallback, this);
	//this->front_image_sub = it.subscribe("/anahita/front_camera/image_raw", 1, &Base_class::imageFrontCallback, this);
	this->bottom_image_sub = it.subscribe("/anahita/bottom_camera/image_raw", 1, &Base_class::imageBottomCallback, this);

	this->enhanced_image_sub = it.subscribe("/anahita/front_camera/preprocessed", 1, &Base_class::fusionCallback, this);
}

void Base_class::imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	try {
        vision_mutex.lock();
		image_front = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        vision_mutex.unlock();
    }
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (cv::Exception &e) {
		ROS_ERROR("cv exception: %s", e.what());
	}
}

void Base_class::imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	try {
		image_bottom = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (cv::Exception &e) {
		ROS_ERROR("cv exception: %s", e.what());
	}
}

void Base_class::fusionCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	try {
		image_front	 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (cv::Exception &e) {
		ROS_ERROR("cv exception: %s", e.what());
	}
}

void Base_class::frontTaskHandling (bool status) {
	if(status)
		spin_thread_front = new boost::thread(boost::bind(&Base_class::spinThreadFront, this)); 
	else {
		close_task = true;
        spin_thread_front->join();
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
		close_task = true;
        spin_thread_bottom->join();
		std::cout << "Bottom Task Handling function over" << std::endl;	
	}
}

void Base_class::spinThreadBottom () {}
void Base_class::spinThreadFront () {}
void Base_class::loadParams () {}
