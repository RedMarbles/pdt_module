
#include "ResamplePreprocessor.hpp"

#include <stdexcept>

#include <sensor_msgs/image_encodings.h>

namespace pdt_module
{

ResamplePreprocessor::ResamplePreprocessor(ros::NodeHandle& _nh, float resample_ratio, int image_type) : 
	resample_ratio_m(resample_ratio), /// Always reduces image size
	image_type_m(image_type)
{
	if(resample_ratio<1)
	{
		throw std::invalid_argument("Attempted to set a resample_ratio less than 1 for ResamplePreprocessor");
	}
}

ResamplePreprocessor::~ResamplePreprocessor()
{
	/// Do nothing
}

void ResamplePreprocessor::processSingleImage(const cv::Mat& image_src, cv::Mat& image_dst)
{
	const int dst_rows = (int)(image_src.rows/resample_ratio_m);
	const int dst_cols = (int)(image_src.cols/resample_ratio_m);
	const cv::Size dst_size( dst_cols, dst_rows );
	image_dst.create( dst_size, image_type_m );
	cv::pyrDown( image_src, image_dst, dst_size );
}

void ResamplePreprocessor::processStereoImage(const cv::Mat& image_left_src, const cv::Mat& image_right_src, cv::Mat& image_left_dst, cv::Mat& image_right_dst)
{
	processSingleImage( image_left_src,  image_left_dst);
	processSingleImage(image_right_src, image_right_dst);
}

} /// exit pdt_module namespace