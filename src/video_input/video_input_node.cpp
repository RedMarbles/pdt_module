// This node implements the next_frame() function and publishes a boost::shared_ptr<boost::gil::rgb8c_view> type 
// It also preprocesses the image

#include "ros/ros.h"
// #include "std_srvs/Empty.h"
// #include "sensor_msgs/Image.h"
// #include "image_transport/image_transport.h"
// #include "cv_bridge/cv_bridge.h"
// #include "sensor_msgs/image_encodings.h"

// #include <opencv2/opencv.hpp>  // OpenCV functions

// #include <boost/gil/gil_all.hpp> // boost::gil functions

// #include "VideoInputPrimitive.hpp"
#include "VideoInputSynchronized.hpp"
// #include "VideoInputVeryPrimitive.hpp"
// #include "VideoTestClass.hpp"

// using namespace pdt_module;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "video_input_node");
	ros::NodeHandle nh_("~");
	ROS_INFO("video_input_node Starting up");

	pdt_module::VideoInputSynchronized video_input_object(nh_, false, true); //Creates publishers and subscribers

	ROS_INFO("video_input_node Initialized");

	try
	{
		ROS_INFO("Starting spin()");
		ros::spin();
	}
	catch (ros::Exception &e)
	{
		ROS_ERROR("Exception caused: %s", e.what());
	}
	return 0;
}