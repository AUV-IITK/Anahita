#include <octagon.h>

Octagon::Octagon () {
    loadParams ();
}

void Octagon::loadParams () {}

void Octagon::spinThreadBottom() {
    ros::Rate loop_rate(15);
	while (ros::ok()) {
		if (task_done) {
			task_done = false;
			break;
		}
		if (!image_bottom.empty()) {
			image_bottom.copyTo(image_marked);
		}
		else ROS_INFO("Image empty");
		ros::spinOnce();
        loop_rate.sleep();
	}
}

void Octagon::spinThreadFront() {
    ros::Rate loop_rate(15);
	while (ros::ok()) {
		if (task_done) {
			task_done = false;
			break;
		}
		if (!image_front.empty())
		{
			image_front.copyTo(image_marked);
		}
		else ROS_INFO("Image empty");
		ros::spinOnce();
        loop_rate.sleep();
	}	
}
