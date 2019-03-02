#include <fusion/laplacianBlend.hpp>

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

void LaplacianBlending::buildPyramids() {
    buildLaplacianPyramid(left, leftLapPyr, leftSmallestLevel);
    buildLaplacianPyramid(right, rightLapPyr, rightSmallestLevel);
    buildGaussianPyramid();
}

void LaplacianBlending::buildGaussianPyramid() {
    assert(leftLapPyr.size() > 0);
    assert(rightLapPyr.size() > 0);

    leftMaskGaussianPyr.clear();
    rightMaskGaussianPyr.clear();

    Mat currRightImg;
    Mat currLeftImg;

    cvtColor(leftMask, currLeftImg, CV_GRAY2BGR);
    cvtColor(rightMask, currRightImg, CV_GRAY2BGR);
    leftMaskGaussianPyr.push_back(currLeftImg); //highest level
    rightMaskGaussianPyr.push_back(currRightImg); //highest level

    currLeftImg = leftMask;
    currRightImg = rightMask;
    for (int l = 1; l < levels+1; l++) {
        Mat _downLeft;
        Mat _downRight;
        if (leftLapPyr.size() > l) {
            pyrDown(currLeftImg, _downLeft, leftLapPyr[l].size());
        }
        else {
            pyrDown(currLeftImg, _downLeft, leftSmallestLevel.size()); //smallest level
        }

        if (rightLapPyr.size() > l) {
            pyrDown(currRightImg, _downRight, rightLapPyr[l].size());
        }
        else {
            pyrDown(currRightImg, _downRight, rightSmallestLevel.size()); //smallest level
        }

        Mat downRight;
        Mat downLeft;
        cvtColor(_downLeft, downLeft, CV_GRAY2BGR);
        cvtColor(_downRight, downRight, CV_GRAY2BGR);
        leftMaskGaussianPyr.push_back(downLeft);
        rightMaskGaussianPyr.push_back(downRight);
        currLeftImg = _downLeft;
        currRightImg = _downRight;
    }
}

void LaplacianBlending::buildLaplacianPyramid(const Mat& img, vector<Mat_<Vec3f> >& lapPyr, Mat& smallestLevel) {
    lapPyr.clear();
    Mat currentImg = img;
    for (int l = 0; l < levels; l++) {
        Mat down,up;
        pyrDown(currentImg, down);
        pyrUp(down, up, currentImg.size());
        Mat lap = currentImg - up;
        lapPyr.push_back(lap);
        currentImg = down;
    }
    currentImg.copyTo(smallestLevel);
}

Mat_<Vec3f> LaplacianBlending::reconstructImgFromLapPyramid() {
    Mat currentImg = resultSmallestLevel;
    for (int l = levels-1; l >= 0; l--) {
        Mat up;

        pyrUp(currentImg, up, resultLapPyr[l].size());
        currentImg = up + resultLapPyr[l];
    }
    return currentImg;
}

void LaplacianBlending::blendLapPyrs() {
    resultSmallestLevel = leftSmallestLevel.mul(leftMaskGaussianPyr.back()) +
    rightSmallestLevel.mul(rightMaskGaussianPyr.back());
    for (int l = 0; l < levels; l++) {
        Mat A = leftLapPyr[l].mul(leftMaskGaussianPyr[l]);
        Mat B = rightLapPyr[l].mul(rightMaskGaussianPyr[l]);
        Mat_<Vec3f> blendedLevel = A + B;

        resultLapPyr.push_back(blendedLevel);
    }
}

LaplacianBlending::LaplacianBlending(const Mat_<Vec3f>& _left, const Mat_<Vec3f>& _right, const Mat_<float>& _leftMask, const Mat_<float>& _rightMask, int _levels):
left(_left), right(_right), rightMask(_rightMask), leftMask(_leftMask), levels(_levels)
{
    assert(_left.size() == _right.size());
    assert(_left.size() == _leftMask.size());
    assert(_rightMask.size() == _leftMask.size());
    buildPyramids();
    blendLapPyrs();
}

Mat_<Vec3f> LaplacianBlending::blend() {
    return reconstructImgFromLapPyramid();
}
