/// Uses services to compute stixels on the image

#include "ros/ros.h"

#include "StixelsEstimatorBasic.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stixels_estimator_node");
	ros::NodeHandle nh_("~");
	ros::Rate rate(100); /// For now set to use 100Hz
	ROS_INFO("stixels_estimator_node Starting up");

	pdt_module::StixelsEstimatorBasic stixels_estimator_object(argc, argv, nh_, true, true);

	ROS_INFO("stixels_estimator_node Initialized");

	try
	{
		ROS_INFO("Starting spin()");
		while(ros::ok())
		{
			ros::spinOnce();
			rate.sleep();
		}
	}
	catch(ros::Exception &e)
	{
		ROS_ERROR("Exception caused: %s", e.what());
	}
	return 0;
}