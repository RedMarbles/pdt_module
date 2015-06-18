
#ifndef _VIDEOINPUTVERYPRIMITIVE_HPP_
#define _VIDEOINPUTVERYPRIMITIVE_HPP_

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
// #include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <opencv2/opencv.hpp>  // OpenCV functions

#include <boost/gil/gil_all.hpp> // boost::gil functions

#define TEST_COMPILE //Comment this if you don't want the test sections of the code to compile

namespace pdt_module
{

class VideoInputVeryPrimitive
{
private:
	ros::NodeHandle _nh;
	ros::Subscriber  left_input_subscriber;
	ros::Subscriber right_input_subscriber;
	ros::Publisher  left_output_publisher;
	ros::Publisher right_output_publisher;
public:
	VideoInputVeryPrimitive();

	~VideoInputVeryPrimitive();

	void left_image_callback(const sensor_msgs::Image& input_left_image);

	void right_image_callback(const sensor_msgs::Image& input_right_image);
#ifdef TEST_COMPILE
	void sampleOutput();
#endif

};

} //exit pdt_module namespace

#endif 
//endblock _VIDEOINPUTPRIMITIVE_HPP_