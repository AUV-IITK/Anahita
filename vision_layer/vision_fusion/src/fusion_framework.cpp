#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <fusion/color_constancy.hpp>
#include <fusion/laplacianBlend.hpp>

using namespace std;
using namespace cv;

int mode = 1;
int fps = 30;
int k = 0;
Mat image;
void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
/**
 * @brief this function generates the exposedness weight map
 * @param input image 1
 * @param input image 2
 * @return output image is weight map
 */
Mat generateEMasks(Mat s1){

  // mean and vairance of exposedness mask
  float _gMean = 0.5;
  float _gVariance = 0.25;

  Mat dst;
  Mat src;
  
  s1.copyTo(src);

  src.convertTo(src, CV_32FC3, 1.0, 0); // covert to float32 data type

  cvtColor(src, src, CV_BGR2HSV_FULL);

  src.convertTo(src, CV_32FC3, 1.0/255.0, 0);

  src.copyTo(dst);

  std::vector<cv::Mat_<float> > ichan;
  std::vector<cv::Mat_<float> > dchan;

  cv::split(src, ichan);
  cv::split(dst, dchan);

  int idx = 2;

  cv::add(ichan[idx], -_gMean, dchan[idx]);
  multiply(dchan[idx], dchan[idx], dchan[idx], 1.0/(2*_gVariance*_gVariance));
  cv::exp(-dchan[idx], dchan[idx]);

  cvtColor(src, src, CV_HSV2BGR_FULL);
  dchan[idx].convertTo(dchan[idx], CV_32FC1,1.0,0);

  return dchan[idx];
}

/**
 * @brief function to compute local contrast
 * @param input image is BGR image
 * @return ouput image is a scalar weight map
 */
Mat localContrast(Mat s)
{
  Mat image;
  s.copyTo(image);

  Mat dst;
  cvtColor(image, image, CV_BGR2YCrCb);
  image.convertTo(image, CV_32FC3, 1.0/255.0, 0);
  std::vector<cv::Mat_<float> > d;
  cv::split(image, d);

  Mat tmp;
  d[0].copyTo(tmp);
  cv::Laplacian(d[0], d[0], image.depth(), 3, 1, 0);
  cv::absdiff(d[0], tmp, d[0]);
  d[0].convertTo(d[0], CV_32FC1, 1.0, 0);

  return d[0];
}

/**
 * @brief function to compute the saliency weight map
 * @param input image is BGR image
 * @return  ouput image is scalar weight map
 */
Mat saliency(Mat s)
{

  Mat image;
  s.copyTo(image);
  Mat i, o1, o2, o3;
  cvtColor(image, image, CV_BGR2Lab);
  cv::Scalar mean = cv::mean(image);
  image.copyTo(i);
  cv::medianBlur(i, i, 9);

  i.convertTo(i, CV_32FC3, 1.0/255.0, 0);
  std::vector<cv::Mat_<float> > d;
  cv::split(i, d);
  cv::add(d[0], -mean[0]/255.0, o1);
  cv::multiply(o1, o1, o1, 1.0);
  cv::add(d[1], -mean[1]/255.0, o2);
  cv::multiply(o2, o2, o2, 1.0);
  cv::add(d[2], -mean[2]/255.0, o3);
  cv::multiply(o3, o3, o3, 1.0);
  cv::add(o1, o2, o1);
  cv::add(o1, o3, o3);
  cv::pow(o3, 0.5 ,o1);

  return o1;

}

/**
 * @brief function to calculate the global contrast weight map for image
 * @param image is input image
 * @return output image is weight map
 */
Mat globalContrast(Mat s)
{

  Mat image;
  s.copyTo(image);
  std::vector<cv::Mat_<float> > d;

  cvtColor(image, image, CV_BGR2Lab);
  image.convertTo(image, CV_32FC3, 1.0/255.0, 0);
  cv::split(image, d);

   Mat blurred, diff;
   double sigma = 7, threshold = 0;
   cv::GaussianBlur(d[0], blurred, Size(), sigma, sigma);

   d[0].convertTo(d[0], CV_32F, 1.0, 0);

   blurred.convertTo(blurred, CV_32F, 1.0, 0);

   cv::absdiff(d[0], blurred, diff);

   return diff;
}

/**
 * @brief this function call the different functions to perform image fusion
 * @param argc
 * @param argv
 * @return
 */

void imageFusionCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	try
	{
    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (cv::Exception &e)
	{
		ROS_ERROR("cv exception: %s", e.what());
	}
} 

int main(int argc, char **argv)
{
  color_correction::contrast_stretching contrast_strech;
  color_correction::gray_world gray_world_;

  Mat a, frame;
  ros::init(argc, argv, "fusion_node");
  ROS_INFO("Starrting");

  ros::NodeHandle nh;
 	image_transport::ImageTransport it(nh);

  image_transport::Subscriber image_raw_sub = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageFusionCallback);
  image_transport::Publisher image_final_pub = it.advertise("/front_camera/preprocessed", 1);
 	ros::Rate loop_rate(25);

  while(ros::ok())
  {
    if(!image.empty())
    {
      // image = cv::imread(argv[1], 1);

      clock_t begin = clock();

      a.create(240, 320, CV_8UC(3));
      resize(image, a, a.size(), 0, 0, INTER_NEAREST);

      Mat cs = contrast_strech.run(a);
      Mat gw = gray_world_.run2(a, 6, 0);

     // imshow("original_image", a);
      //imshow("input_1", gw);

      Mat c1 = generateEMasks(a);

     {
        Mat global_weight_gw = globalContrast(gw);
        Mat global_weight_cs = globalContrast(cs);
        Mat saliency_weight_cs = saliency(cs);
        Mat saliency_weight_gw = saliency(gw);
        Mat exposedness_weight_cs = generateEMasks(cs);
        Mat exposedness_weight_gw = generateEMasks(gw);
        Mat local_weight_cs = localContrast(cs);
        Mat local_weight_gw = localContrast(gw);

        Mat sum;
        c1.copyTo(sum);

        cs.convertTo(cs, CV_32FC3, 1.0/255.0);
        gw.convertTo(gw, CV_32FC3, 1.0/255.0);
        Mat w1, w2;

        ////////////////////// global contrast ////////////////////////
       /* {
          Mat result1;
          cv::add(global_weight_cs, global_weight_gw, sum);

          cv::divide(global_weight_cs, sum, w1);
          cv::divide(global_weight_gw, sum, w2);

          LaplacianBlending lBlend(cs, gw, w1, w2, 4);
          result1 = lBlend.blend();

          result1.convertTo(result1, CV_8U, 255.0, 0);
          //imshow("global contrast", result1);
        }
        
        ////////////////////// saliency ////////////////////////
        {
          Mat result2;
          cv::add(saliency_weight_cs, saliency_weight_gw, sum);

          cv::divide(saliency_weight_cs, sum, w1);
          cv::divide(saliency_weight_gw, sum, w2);

          LaplacianBlending lBlend(cs, gw, w1, w2, 4);
          result2 = lBlend.blend();

          result2.convertTo(result2, CV_8U, 255.0, 0);
          //imshow("saliency", result2);
        }
        ////////////////////// exposedness ////////////////////////    
        {
          Mat result3;
          cv::add(exposedness_weight_cs, exposedness_weight_gw, sum);

          cv::divide(exposedness_weight_cs, sum, w1);
       r   cv::divide(exposedness_weight_gw, sum, w2);

          LaplacianBlending lBlend(cs, gw, w1, w2, 4);
          result3=lBlend.blend();

          result3.convertTo(result3, CV_8U, 255.0, 0);
          //imshow("exposedness", result3);
        }
        ////////////////////// local contrast ////////////////////////
        {
          Mat result4;
          cv::add(local_weight_cs, local_weight_gw, sum);

          cv::divide(local_weight_cs, sum, w1);
          cv::divide(local_weight_gw, sum, w2);

          LaplacianBlending lBlend(cs, gw, w1, w2, 4);
          result4 = lBlend.blend();

          result4.convertTo(result4, CV_8U, 255.0, 0);
        //  imshow("local contrast", result4);
        }
        */
        cv::add(c1, global_weight_gw, sum);
        cv::add(sum, saliency_weight_cs, sum);
        cv::add(sum, saliency_weight_gw, sum);
        cv::add(sum, exposedness_weight_cs, sum);
        cv::add(sum, exposedness_weight_gw, sum);
        cv::add(sum, local_weight_cs, sum);
        cv::add(sum, local_weight_gw, sum);

        cv::divide(c1, sum, c1);
        cv::divide(global_weight_gw, sum, global_weight_gw);
        cv::divide(saliency_weight_cs, sum, saliency_weight_cs);
        cv::divide(saliency_weight_gw, sum, saliency_weight_gw);
        cv::divide(exposedness_weight_cs, sum, exposedness_weight_cs);
        cv::divide(exposedness_weight_gw, sum, exposedness_weight_gw);
        cv::divide(local_weight_cs, sum, local_weight_cs);
        cv::divide(local_weight_gw, sum, local_weight_gw);

        cvtColor(c1, c1, CV_GRAY2BGR);
        cvtColor(global_weight_gw, global_weight_gw, CV_GRAY2BGR);
        cvtColor(saliency_weight_cs, saliency_weight_cs, CV_GRAY2BGR);
        cvtColor(saliency_weight_gw, saliency_weight_gw, CV_GRAY2BGR);
        cvtColor(exposedness_weight_cs, exposedness_weight_cs, CV_GRAY2BGR);
        cvtColor(exposedness_weight_gw, exposedness_weight_gw, CV_GRAY2BGR);
        cvtColor(local_weight_cs, local_weight_cs, CV_GRAY2BGR);
        cvtColor(local_weight_gw, local_weight_gw, CV_GRAY2BGR);

        cv::multiply(c1, cs, c1);
        cv::multiply(saliency_weight_cs, cs, saliency_weight_cs);
        cv::multiply(exposedness_weight_cs, cs, exposedness_weight_cs);
        cv::multiply(local_weight_cs, cs, local_weight_cs);
        cv::multiply(global_weight_gw, gw, global_weight_gw);
        cv::multiply(saliency_weight_gw, gw, saliency_weight_gw);
        cv::multiply(exposedness_weight_gw, gw, exposedness_weight_gw);
        cv::multiply(local_weight_gw, gw, local_weight_gw);

        cv::add(c1, global_weight_gw, c1);
        cv::add(c1, saliency_weight_cs, c1);
        cv::add(c1, exposedness_weight_cs, c1);
        cv::add(c1, local_weight_cs, c1);
        cv::add(c1, saliency_weight_gw, c1);
        cv::add(c1, exposedness_weight_gw, c1);
        cv::add(c1, local_weight_gw, c1);
        {
          c1.convertTo(c1, CV_8U, 255.0, 0);
          //imshow("naive blend", c1);
        }
        //cv::waitKey(0);
        image_final_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", c1).toImageMsg());

      }
    }
    else
    {
      ROS_INFO("Image is empty");
    }
    loop_rate.sleep();
		ros::spinOnce();      
  }
}
