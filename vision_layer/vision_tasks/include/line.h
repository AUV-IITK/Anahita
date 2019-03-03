#ifndef LINE_TASK_H
#define LINE_TASK_H

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
// #include <geometry_msgs/Pose2D.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Bool.h>
// #include <sensor_msgs/image_encodings.h>
// #include <bits/stdc++.h>
// #include <stdlib.h>
// #include <string>

// #include <boost/thread.hpp> 

#include <vision_tasks/lineRangeConfig.h>
// #include <vision_commons/filter.h>
// #include <vision_commons/contour.h>
// #include <vision_commons/morph.h>
// #include <vision_commons/threshold.h>

#include "base_class.h"

class Line: public Base_class
{
protected:
	
	void callback(vision_tasks::lineRangeConfig &config, double level);
    double computeMean(std::vector<double> &newAngles);

public:
    Line();

};
#endif // LINE_TASK_H

