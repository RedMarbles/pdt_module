// Does nothing at all, for a dummy preprocessor object.

#ifndef _PDT_EMPTYPREPROCESSOR_HPP_
#define _PDT_EMPTYPREPROCESSOR_HPP_

#include "AbstractPreprocessor.hpp"

#include <cstdlib>

//OpenCV
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>


namespace pdt_module
{

class EmptyPreprocessor : public AbstractPreprocessor
{
protected:
	/// no members

public:
	EmptyPreprocessor(ros::NodeHandle& _nh)
	{
		/// Do nothing
	}

	~EmptyPreprocessor()
	{
		/// Do nothing
	}

	void processSingleImage(const cv::Mat& image_src, cv::Mat& image_dst)
	{
		image_dst = image_src;
	}

	void processStereoImage(const cv::Mat& image_left_src, const cv::Mat& image_right_src, cv::Mat& image_left_dst, cv::Mat& image_right_dst)
	{
		image_left_dst  = image_left_src;
		image_right_dst = image_right_src;
	}
};


} // exit pdt_module namespace



#endif // _PDT_EMPTYPREPROCESSOR_HPP_