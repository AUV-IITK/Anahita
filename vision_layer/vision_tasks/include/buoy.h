#ifndef BUOY_TASK_H
#define BUOY_TASK_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <string>
#include <boost/thread.hpp> 

#include <vision_tasks/buoyRangeConfig.h>
#include <vision_commons/filter.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>

#include "base_class.h"

class Buoy: public Base_class
{
protected:
	int data_low_h[3] = {0, 12, 0};
	int data_high_h[3] = {17, 40, 56};
	int data_low_s[3] = {206, 183, 0};
	int data_high_s[3] = {255, 255, 255};
	int data_low_v[3] = {30, 3, 2};
	int data_high_v[3] = {255, 255, 82};



	void callback(vision_tasks::buoyRangeConfig &config, double level);
	

public:
	Buoy();

};
#endif // BUOY_TASK_H
