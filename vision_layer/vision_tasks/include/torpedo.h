#ifndef TORPEDO_TASK_H
#define TORPEDO_TASK_H

// #include "ros/ros.h"
// #include "sensor_msgs/Image.h"
// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/core/core.hpp"
// #include "opencv2/imgproc/imgproc_c.h"
// #include "opencv2/highgui/highgui.hpp"
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <dynamic_reconfigure/server.h>
// #include <geometry_msgs/PointStamped.h>
// #include <sensor_msgs/image_encodings.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
// #include <bits/stdc++.h>
// #include <stdlib.h>
// #include <string>
// #include <boost/thread.hpp> 

#include <vision_tasks/torpedoRangeConfig.h>
// #include <vision_commons/filter.h>
// #include <vision_commons/contour.h>
// #include <vision_commons/morph.h>
// #include <vision_commons/threshold.h>

#include "base_class.h"

class Torpedo: public Base_class
{
protected:

	int data_low_h[2] = {43, 0};
	int data_high_h[2] = {74, 13};
	int data_low_s[2] = {41, 220};
	int data_high_s[2] = {255, 255};
	int data_low_v[2] = {0, 87};
	int data_high_v[2] = {255, 181};

	void callback(vision_tasks::torpedoRangeConfig &config, double level);
	

public:
    Torpedo();

};
#endif // TORPEDO_TASK_H

