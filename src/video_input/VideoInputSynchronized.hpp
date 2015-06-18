
#ifndef _VIDEOINPUTSYNCHRONIZED_HPP_
#define _VIDEOINPUTSYNCHRONIZED_HPP_

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
#include "pdt_module/FetchStereoImages.h"
//#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include <opencv2/opencv.hpp>  // OpenCV functions

#include <boost/gil/gil_all.hpp> // boost::gil functions

namespace pdt_module
{

class VideoInputSynchronized
{
private:
	ros::NodeHandle _nh;
	message_filters::Subscriber<sensor_msgs::Image> left_input_subscriber;
	message_filters::Subscriber<sensor_msgs::Image> right_input_subscriber;
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> synced_subscriber;
	sensor_msgs::ImagePtr left_image_p;
	sensor_msgs::ImagePtr right_image_p;
	ros::ServiceServer load_next_frame_service;
	ros::ServiceServer fetch_stereo_frame_service;
	bool flag_load_next_frame;

public:
	VideoInputSynchronized();

	~VideoInputSynchronized();

	void stereo_image_callback(const sensor_msgs::ImageConstPtr& input_left_image, const sensor_msgs::ImageConstPtr& input_right_image);

	bool load_next_frame(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	bool fetch_stereo_frame(pdt_module::FetchStereoImages::Request& req, pdt_module::FetchStereoImages::Response& res);
};

} //exit pdt_module namespace

#endif 
//endblock _VIDEOINPUTSYNCHRONIZED_HPP_