// Class for a Mei camera model preprocessor for undistorting

#ifndef _PDT_MEIMODELPREPROCESSOR_HPP_
#define _PDT_MEIMODELPREPROCESSOR_HPP_

#include "AbstractPreprocessor.hpp"

#include <cstdlib>

//OpenCV
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>


namespace pdt_module
{

class MeiModelPreprocessor : public AbstractPreprocessor
{
protected:
	std::string camera_type;  /// Should be "mei"

	cv::Mat mapl1, mapl2, mapr1, mapr2; /// Remapping matrices

public:
	MeiModelPreprocessor(ros::NodeHandle& _nh);

	~MeiModelPreprocessor();

	void processSingleImage(const cv::Mat& image_src, cv::Mat& image_dst);

	void processStereoImage(const cv::Mat& image_left_src, const cv::Mat& image_right_src, cv::Mat& image_left_dst, cv::Mat& image_right_dst);
};


} // exit pdt_module namespace



#endif // _PDT_MEIMODELPREPROCESSOR_HPP_