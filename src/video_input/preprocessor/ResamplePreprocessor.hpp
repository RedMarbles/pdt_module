// Class for resampling the image to a different size

#ifndef _PDT_RESAMPLEPREPROCESSOR_HPP_
#define _PDT_RESAMPLEPREPROCESSOR_HPP_

#include "AbstractPreprocessor.hpp"

#include <cstdlib>

//OpenCV
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>


namespace pdt_module
{

class ResamplePreprocessor : public AbstractPreprocessor
{
protected:
	const float resample_ratio_m;
	const int image_type_m;

public:
	ResamplePreprocessor(ros::NodeHandle& _nh, float resample_ratio = 2.0, int image_type = CV_8UC3);

	~ResamplePreprocessor();

	void processSingleImage(const cv::Mat& image_src, cv::Mat& image_dst);

	void processStereoImage(const cv::Mat& image_left_src, const cv::Mat& image_right_src, cv::Mat& image_left_dst, cv::Mat& image_right_dst);
};


} // exit pdt_module namespace



#endif // _PDT_RESAMPLEPREPROCESSOR_HPP_