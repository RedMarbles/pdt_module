// This node implements the next_frame() function and publishes a boost::shared_ptr<boost::gil::rgb8c_view> type 

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/Empty.h"

#include <cstdlib>
#include <iostream>

#include <boost/gil/gil_all.hpp>

ros::Publisher left_image_publisher;
ros::Publisher right_image_publisher;

bool fetch_next_left_image;
bool fetch_next_right_image;


void left_image_callback(const sensor_msgs::Image& left_image)
{
	//ROS_INFO("Left Image encoding : %s", left_image.encoding.c_str());
	//The encoding is "bgr8"

	//if(gui_initialized == false) init_gui(left_image.width, left_image.height);

	//convert_sensor_to_gil(left_image, screen_left_view);

	//update_gui();
	if(fetch_next_left_image)
	{
		//Preprocess left_image first
		left_image_publisher.publish(left_image);
		fetch_next_left_image = false;
	}

	return;
}

void right_image_callback(const sensor_msgs::Image& right_image)
{
	//ROS_INFO("Right Image encoding : %s", right_image.encoding.c_str());
	
	//if( !gui_initialized ) init_gui(right_image.width, right_image.height);

	//convert_sensor_to_gil(right_image, screen_right_view);

	//update_gui();

	if(fetch_next_right_image)
	{
		//Preprocess right_image first
		right_image_publisher.publish(right_image);
		fetch_next_right_image = false;
	}

	return;
}

bool next_frame(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	if(fetch_next_left_image || fetch_next_right_image)
	{
		ROS_ERROR("Still has not completed fetching the previous images");
		return false;
	}
	else
	{
		fetch_next_right_image = true;
		fetch_next_left_image = true;
	}
	return true;
}


int main(int argc, char** argv)
{
	//ROS setup
	ros::init(argc, argv, "read_from_camera_node");
	ros::NodeHandle _nh("~");

	ROS_DEBUG("read_from_camera_node Initialized");

	ros::Subscriber left_image_subscriber = _nh.subscribe("left_image",1,left_image_callback);
	ros::Subscriber right_image_subscriber = _nh.subscribe("right_image",1,right_image_callback);

	left_image_publisher = _nh.advertise<sensor_msgs::Image>("capture_left_frame",1);
	right_image_publisher = _nh.advertise<sensor_msgs::Image>("capture_right_frame",1);
	
	ros::ServiceServer next_frame_service = _nh.advertiseService("next_frame_service",next_frame);

	fetch_next_right_image = false;
	fetch_next_left_image = false;

	ROS_DEBUG("read_from_camera_node Ready to process requests");

	ros::spin();

	return 0;
}