// Uses services to load image from video_input_node

#include "ros/ros.h"

#include "ObjectDetectorBasic.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "object_detector_node");
	ROS_INFO("object_detector_node Starting up");

	pdt_module::ObjectDetectorBasic object_detector(argc, argv); // Creates publishers and subscribers on initialization

	ROS_INFO("object_detector_node Initialized");

	object_detector.main_loop();

	// try
	// {
	// 	ROS_INFO("Starting spin()");
	// 	ros::spin();
	// }
	// catch( ros::Exception &e )
	// {
	// 	ROS_ERROR("Exception caused: %s", e.what());
	// }
	return 0;
}