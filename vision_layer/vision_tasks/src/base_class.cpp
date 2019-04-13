#include <base_class.h>

Base_class::Base_class(){
	init();
}

void Base_class::init(){
	image_transport::ImageTransport it(nh);

	
	this->front_thresholded_pub = it.advertise("/anahita/vision/front/thresholded", 1);
	this->front_marked_pub = it.advertise("/anahita/vision/front/marked", 1);
	
    this->bottom_thresholded_pub = it.advertise("/anahita/vision/bottom/thresholded", 1);
    this->bottom_marked_pub = it.advertise("/anahita/vision/bottom/marked", 1);

	this->front_x_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/vision/front/x_coordinate", 1000);
	this->front_y_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/vision/front/y_coordinate", 1000);
	this->front_z_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/vision/front/z_coordinate", 1000);
	this->bottom_x_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/vision/bottom/x_coordinate", 1000);
	this->bottom_y_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/vision/bottom/y_coordinate", 1000);
	this->bottom_z_coordinates_pub = nh.advertise<std_msgs::Float32>("/anahita/vision/bottom/z_coordinate", 1000);

	this->detection_pub = nh.advertise<std_msgs::Bool>("/detected", 1000);
}

void Base_class::frontTaskHandling (bool status) {
	if(status)
	{
		spin_thread_front = new boost::thread(boost::bind(&Base_class::spinThreadFront, this)); 
	}
	else 
	{
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

void Base_class::spinThreadBottom(){}

void Base_class::spinThreadFront(){}
