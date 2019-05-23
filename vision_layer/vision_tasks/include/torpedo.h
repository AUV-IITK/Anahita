#ifndef TORPEDO_TASK_H
#define TORPEDO_TASK_H

#include "base_class.h"
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

class Torpedo : public Base_class
{
	public:
		Torpedo();
		~Torpedo();
		virtual void loadParams () override;
		virtual void spinThreadFront () override;
		virtual void spinThreadBottom () override;
        void findCircles (cv::Mat& src_img, cv::Mat& thres_img, double circle_threshold);
        void InitTracker (cv::Mat& src_img, cv::Mat& thres_img, double circle_threshold);
        void updateTracker (cv::Mat& src_img);
    private:
        image_transport::Publisher front_roi_pub;
        cv::Mat marked_img;
        bool initTracker = false;
        
        cv::Rect2d bbox1;
        cv::Rect2d bbox2;
        cv::Rect2d bbox3;
        
        cv::Ptr<cv::Tracker> tracker1;
        cv::Ptr<cv::Tracker> tracker2;
        cv::Ptr<cv::Tracker> tracker3;
};
#endif // TORPEDO_TASK_H

