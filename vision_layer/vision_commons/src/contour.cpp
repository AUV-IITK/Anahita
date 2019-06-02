#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <string>
#include <iostream>
#include "contour.h"

std::vector<std::vector<cv::Point>> vision_commons::Contour::getBestX(cv::Mat &raw, int x)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(raw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
    if (contours.size() != 0)
    {
        std::vector<float> areas;
        for (int i = 0; i < contours.size(); i++)
        {
            float area = cv::contourArea(contours[i]);
            if (area == 0.0)
                contours.erase(contours.begin() + i--);
            else
                areas.push_back(cv::contourArea(contours[i]));
        }
        float max_area = 0.0f;
        int max_index = 0;
        float area = 0.0f;
        int iter = 0;
        if (x < areas.size())
            iter = x;
        else
            iter = areas.size();
        for (int i = 0; i < iter; i++)
        {
            max_area = areas[i];
            max_index = i;
            for (int j = i + 1; j < contours.size(); j++)
            {
                area = areas[j];
                if (area > max_area)
                {
                    max_area = area;
                    max_index = j;
                }
            }
            std::vector<cv::Point> temp = contours[i];
            contours[i] = contours[max_index];
            contours[max_index] = temp;
            float tempArea = areas[i];
            areas[i] = areas[max_index];
            areas[max_index] = tempArea;
            max_area = 0.0;
            area = 0.0;
            max_index = 0;
        }

        if (areas.size() > x)
        {
            std::vector<std::vector<cv::Point>> topX(contours.begin(), contours.begin() + x);
            return topX;
        }
        else
            return contours;
    }
    else
        return contours;
}

std::vector<std::vector<cv::Point>> vision_commons::Contour::getBestXConvexHulled(cv::Mat &raw, int x)
{
    std::vector<std::vector<cv::Point>> contours = getBestX(raw, x);
    std::vector<std::vector<cv::Point>> hull(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        cv::convexHull(cv::Mat(contours[i]), hull[i], false);
    }
    return hull;
}

std::vector<cv::Point> vision_commons::Contour::getLargestContour(cv::Mat &raw)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(raw, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
    double largest_area = 0, largest_contour_index = 0;

    for (int i = 0; i < contours.size(); i++) // iterate through each contour.
    {
        double a = contourArea(contours[i], false); //  Find the area of contour
        if (a > largest_area)
        {
            largest_area = a;
            largest_contour_index = i; // Store the index of largest contour
        }
    }

    if (contours.size() == 0) {
        std::vector<cv::Point> empty_contour;
        return empty_contour;
    }

    return contours[largest_contour_index];
}

void vision_commons::Contour::filterContours (const std::vector<std::vector<cv::Point> >& contours, std::vector<int>& idx, double threshold) {
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea (contours[i], false) < threshold) continue;
        idx.push_back (i);
    }
}

bool contour_cmp (std::vector<cv::Point> contour_a, std::vector<cv::Point> contour_b) {
    return cv::contourArea (contour_a, false) > cv::contourArea (contour_b, false);
}

void vision_commons::Contour::sortFromBigToSmall (std::vector<std::vector<cv::Point> >& contours) {
    if (contours.size() == 0) return;
    std::sort (contours.begin(), contours.end(), contour_cmp);
}

std::vector<std::vector<cv::Point> > vision_commons::Contour::filterContours (const std::vector<std::vector<cv::Point> >& contours, double threshold) {
    std::vector<std::vector<cv::Point> > filtered_contours;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea (contours[i], false) < threshold) continue;
        filtered_contours.push_back (contours[i]);
    }
    return filtered_contours;
}
