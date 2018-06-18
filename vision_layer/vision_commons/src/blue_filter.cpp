#include "opencv2/photo/photo.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include "vision_commons/blue_filter.h"

void balance_white(cv::Mat mat) {
  double discard_ratio = 0.05;
  int hists[3][256];
  memset(hists, 0, 3*256*sizeof(int));

  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        hists[j][ptr[x * 3 + j]] += 1;
      }
    }
  }

  // cumulative hist
  int total = mat.cols*mat.rows;
  int vmin[3], vmax[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 255; ++j) {
      hists[i][j + 1] += hists[i][j];
    }
    vmin[i] = 0;
    vmax[i] = 255;
    while (hists[i][vmin[i]] < discard_ratio * total)
      vmin[i] += 1;
    while (hists[i][vmax[i]] > (1 - discard_ratio) * total)
      vmax[i] -= 1;
    if (vmax[i] < 255 - 1)
      vmax[i] += 1;
  }


  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        int val = ptr[x * 3 + j];
        if (val < vmin[j])
          val = vmin[j];
        if (val > vmax[j])
          val = vmax[j];
        ptr[x * 3 + j] = static_cast<uchar>((val - vmin[j]) * 255.0 / (vmax[j] - vmin[j]));
      }
    }
  }
}

cv::Mat vision_commons::BlueFilter::filter(
	cv::Mat image,
	double clahe_clip,
	int clahe_grid_size,
	int clahe_bilateral_iter,
	int balanced_bilateral_iter,
	double denoise_h
) {

	if(!image.empty()){
		cv::Mat lab_image;
		cv::cvtColor(image, lab_image, CV_BGR2Lab);
		std::vector<cv::Mat> lab_planes(3);
		cv::split(lab_image, lab_planes);
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clahe_clip, cv::Size(clahe_grid_size, clahe_grid_size));
		cv::Mat l0dst;
		clahe->apply(lab_planes[0], l0dst);
		l0dst.copyTo(lab_planes[0]);
		cv::merge(lab_planes, lab_image);
		cv::Mat blue_filtered;
		cv::cvtColor(lab_image, blue_filtered, CV_Lab2BGR);
		cv::Mat temp;
		for(int i = 0 ; i < clahe_bilateral_iter/2 ; i++) {
			cv::bilateralFilter(blue_filtered, temp, 6, 8.0, 8.0);
			cv::bilateralFilter(temp, blue_filtered, 6, 8.0, 8.0);
		}
		balance_white(blue_filtered);
		for(int i = 0 ; i < balanced_bilateral_iter/2 ; i++) {
			cv::bilateralFilter(blue_filtered, temp, 6, 8.0, 8.0);
			cv::bilateralFilter(temp, blue_filtered, 6, 8.0, 8.0);
		}
		cv::fastNlMeansDenoisingColored(blue_filtered, blue_filtered, denoise_h, denoise_h, 3, 5);

		// cv::Mat noise_free;
		// cv::bilateralFilter(image, noise_free, 1, 2.0, 0.5);
		// cv::Mat lab_image;
		// cv::cvtColor(noise_free, lab_image, CV_BGR2Lab);
		// std::vector<cv::Mat> lab_planes(3);
		// cv::split(lab_image, lab_planes);
		// cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(claheClip, cv::Size(claheGridSize, claheGridSize));
		// cv::Mat l0dst;
		// clahe->apply(lab_planes[0], l0dst);
		// l0dst.copyTo(lab_planes[0]);
		// cv::merge(lab_planes, lab_image);
		// cv::Mat histogram_equalized;
		// cv::cvtColor(lab_image, histogram_equalized, CV_Lab2BGR);
		// cv::Mat white_balanced;
		// white_balanced = whiteBalance(image);
		// cv::Mat blue_filtered;
		// cv::addWeighted(white_balanced, whiteBalanceWeight, histogram_equalized, 1.0-whiteBalanceWeight, 0.0, blue_filtered);
		// cv::fastNlMeansDenoisingColored(blue_filtered, blue_filtered, denoiseH, denoiseH, 3, 5);
		return blue_filtered;
	}
	else
		return image;
}
