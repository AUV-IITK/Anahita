#ifndef TORPEDO_TASK_H
#define TORPEDO_TASK_H

#include "base_class.h"
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <master_layer/ChangeTorpedoHole.h>

class Torpedo : public Base_class
{
	public:
		Torpedo();
		virtual void loadParams () override;
		virtual void spinThreadFront () override;
        void findCircles (cv::Mat& src_img, cv::Mat& thres_img, double circle_threshold);
        void InitTracker (cv::Mat& src_img, cv::Mat& thres_img, double circle_threshold);
        void updateTracker (cv::Mat& src_img);
        cv::Point2f threshROI (const cv::Rect2d& bounding_rect, const cv::Mat& img, int padding);
        void recogniseHoles (cv::Mat& thres_img);
        void updateCoordinates (std::vector<cv::Point> points);
        bool changeTorpedoHole (master_layer::ChangeTorpedoHole::Request &req,
                                master_layer::ChangeTorpedoHole::Response &resp);

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

        cv::Point TR;
        cv::Point TL;
        cv::Point BOT;
        
        bool TR_init = false;
        bool TL_init = false;
        bool BOT_init = false;

        std::string current_hole = "";
        ros::ServiceServer service;
};
#endif // TORPEDO_TASK_H

