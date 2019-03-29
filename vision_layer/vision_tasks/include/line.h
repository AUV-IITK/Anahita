#ifndef LINE_TASK_H
#define LINE_TASK_H

#include "base_class.h"

#include <vision_tasks/lineRangeConfig.h>

class Line : public Base_class
{
protected:

    image_transport::Subscriber image_raw_sub;

   	ros::Publisher detection_pub;
	ros::Publisher coordinates_pub;
	
	std::string camera_frame_;

	bool task_done = false;

	ros::Publisher ang_pub;
	
	void callback(vision_tasks::lineRangeConfig &config, double level);
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
    double computeMean(std::vector<double> &newAngles);

public:
    Line();
    void spinThreadBottom();
};
#endif // LINE_TASK_H

