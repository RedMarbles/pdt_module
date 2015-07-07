// Class for abstracting preprocessors

#ifndef _PDT_ABSTRACTPREPROCESSOR_HPP_
#define _PDT_ABSTRACTPREPROCESSOR_HPP_

#include <cstdlib>

//OpenCV
#include <opencv2/opencv.hpp>

namespace pdt_module
{

class AbstractPreprocessor
{
protected:
	/// No members

public:
	AbstractPreprocessor();

	~AbstractPreprocessor();

	virtual void processSingleImage(const cv::Mat& image_src, cv::Mat& image_dst) = 0;

	virtual void processStereoImage(const cv::Mat& image_left_src, const cv::Mat& image_right_src, cv::Mat& image_left_dst, cv::Mat& image_right_dst) = 0;
};


} // exit pdt_module namespace



#endif // _PDT_ABSTRACTPREPROCESSOR_HPP_