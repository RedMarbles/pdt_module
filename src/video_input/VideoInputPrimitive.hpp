
#ifndef _VIDEOINPUTPRIMITIVE_HPP_
#define _VIDEOINPUTPRIMITIVE_HPP_

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <opencv2/opencv.hpp>  // OpenCV functions

#include <boost/gil/gil_all.hpp> // boost::gil functions

namespace pdt_module
{

class VideoInputPrimitive
{
private:
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;
	image_transport::Subscriber  left_input_subscriber;
	image_transport::Subscriber right_input_subscriber;
	image_transport::Publisher  left_output_publisher;
	image_transport::Publisher right_output_publisher;
public:
	VideoInputPrimitive();

	~VideoInputPrimitive();

	void left_image_callback(const sensor_msgs::ImageConstPtr& input_left_image);

	void right_image_callback(const sensor_msgs::ImageConstPtr& input_right_image);

};

} //exit pdt_module namespace

#endif 
//endblock _VIDEOINPUTPRIMITIVE_HPP_