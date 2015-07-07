
#include "MeiModelPreprocessor.hpp"

#include <stdexcept>

#include "camera_models/utils.h"
#include "camera_models/utils_mei.h"

#include <sensor_msgs/image_encodings.h>

namespace pdt_module
{

MeiModelPreprocessor::MeiModelPreprocessor(ros::NodeHandle& _nh)
{
	camera_type = "mei";

	cv::Mat R1, R2; /// External calibration matrices

	std::string info_left_filename;
	std::string info_right_filename;
	std::string info_ext_filename;
	std::string info_ext_typeCamera;

	/// Dummy variables, not used right now.
	sensor_msgs::CameraInfo info_cam_left;
	sensor_msgs::CameraInfo info_cam_right;

	_nh.getParam("param_left",info_left_filename);
	_nh.getParam("param_right",info_right_filename);
	_nh.getParam("param_ext",info_ext_filename);
	_nh.getParam("camera_type",info_ext_typeCamera);

	if(info_ext_typeCamera.compare(camera_type) != 0)
	{
		throw std::runtime_error("Incorrect camera model");
	}

	MEI::parameters paramsLeft;   
	MEI::parameters paramsRight;

	MEI::loadIntrinsicFromYAML(info_left_filename.c_str(),info_cam_left, paramsLeft);
	MEI::loadIntrinsicFromYAML(info_right_filename.c_str(),info_cam_right, paramsRight);
	MEI::loadExtrinsicFromYAML(info_ext_filename.c_str(),info_cam_left, info_cam_right, R1, R2);

	/// function to undistort the image. build map1 and map2
	MEI::initUndistortRectifyMap(mapl1,mapl2, R1, paramsLeft); 
	MEI::initUndistortRectifyMap(mapr1,mapr2, R2, paramsRight);
}

MeiModelPreprocessor::~MeiModelPreprocessor()
{
	/// Do nothing
}

void MeiModelPreprocessor::processSingleImage(const cv::Mat& image_src, cv::Mat& image_dst)
{
	cv::remap(image_src, image_dst, mapl1, mapl2, cv::INTER_LINEAR);
}

void MeiModelPreprocessor::processStereoImage(const cv::Mat& image_left_src, const cv::Mat& image_right_src, cv::Mat& image_left_dst, cv::Mat& image_right_dst)
{
	cv::remap(image_left_src, image_left_dst, mapl1, mapl2, cv::INTER_LINEAR);
	cv::remap(image_right_src, image_right_dst, mapr1, mapr2, cv::INTER_LINEAR);
}


} /// exit pdt_module namespace