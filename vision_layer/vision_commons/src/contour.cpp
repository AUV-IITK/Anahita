#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <string>
#include "vision_commons/contour.h"

std::vector<std::vector<cv::Point> > vision_commons::Contour::getBestX(cv::Mat raw, int x) {
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(raw, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	if (contours.size() != 0){
		int index = -1;
		float area = 0.0;
		for(int i = 0 ; i < x ; i++) {
			float max_area = cv::contourArea(contours[i]);
			int max_index = i;
			for( int j = i+1; j < contours.size(); i++  ){
				area = cv::contourArea(contours[j]);
				if(area > max_area){
					max_area =area;
					max_index = j;
				}
			}
			std::vector<cv::Point> temp = contours[i];
			contours[i] = contours[max_index];
			contours[max_index] = temp;
		}
		std::vector<std::vector<cv::Point> > topX(contours.begin(), contours.begin() + x);
		return topX;
	}
	else
		return contours;
}

std::vector<std::vector<cv::Point> > vision_commons::Contour::getBestXConvexHulled(cv::Mat raw, int x) {
	std::vector<std::vector<cv::Point> > contours = getBestX(raw, x);
	std::vector<std::vector<cv::Point> > hull(contours.size());
	for( int i = 0; i < contours.size(); i++  ){
		cv::convexHull( cv::Mat(contours[i]), hull[i], false);
	}
	return hull;
}
