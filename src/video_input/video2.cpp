// This node implements the next_frame() function and publishes a boost::shared_ptr<boost::gil::rgb8c_view> type 
// It also preprocesses the image

#include <cstdlib>
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <opencv2/opencv.hpp>  // OpenCV functions


// #include <boost/gil/gil_all.hpp> // boost::gil functions

// #include "VideoInputPrimitive.hpp"

// using namespace pdt_module;
//using namespace std;

static ros::NodeHandle *_nh;
static image_transport::ImageTransport *_it;
static image_transport::Subscriber  left_input_subscriber;
static image_transport::Subscriber right_input_subscriber;
// static image_transport::Publisher  left_output_publisher;
// static image_transport::Publisher right_output_publisher;
bool save_left, save_right;
std::string filename;
int filename_num;


void left_image_callback(const sensor_msgs::ImageConstPtr& input_left_image)
{
	cv_bridge::CvImageConstPtr left_image_cv_ptr;
	cv_bridge::CvImagePtr left_reduced_cv_ptr;
	sensor_msgs::ImageConstPtr output_left_image;
	try
	{
		left_image_cv_ptr = cv_bridge::toCvShare(input_left_image,"");
	}
	catch(cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	const int orig_rows = left_image_cv_ptr->image.rows;
	const int orig_cols = left_image_cv_ptr->image.cols;

	cv::pyrDown( left_image_cv_ptr->image, left_reduced_cv_ptr->image, cv::Size( orig_cols/2, orig_rows/2 ));
	left_reduced_cv_ptr->encoding = left_image_cv_ptr->encoding;
	left_reduced_cv_ptr->header = left_image_cv_ptr->header;

	cv::imshow("Output_Left", left_reduced_cv_ptr->image); //Test code

	// left_output_publisher.publish( left_reduced_cv_ptr->toImageMsg() );

	if(save_left)
	{
		cv::imwrite(filename+std::to_string(filename_num)+"L.png", left_reduced_cv_ptr->image);
		save_left = false;
	}
}

// void right_image_callback(sensor_msgs::ImageConstPtr& input_right_image)
void right_image_callback(const sensor_msgs::ImageConstPtr& input_right_image)
{
	cv_bridge::CvImageConstPtr right_image_cv_ptr;
	cv_bridge::CvImagePtr right_reduced_cv_ptr;
	sensor_msgs::ImageConstPtr output_right_image;
	try
	{
		right_image_cv_ptr = cv_bridge::toCvShare(input_right_image,"");
	}
	catch(cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	const int orig_rows = right_image_cv_ptr->image.rows;
	const int orig_cols = right_image_cv_ptr->image.cols;

	cv::pyrDown( right_image_cv_ptr->image, right_reduced_cv_ptr->image, cv::Size( orig_cols/2, orig_rows/2 ));
	right_reduced_cv_ptr->encoding = right_image_cv_ptr->encoding;
	right_reduced_cv_ptr->header = right_image_cv_ptr->header;

	cv::imshow("Output_Right", right_reduced_cv_ptr->image); //Test code

	// right_output_publisher.publish( right_reduced_cv_ptr->toImageMsg() );

	if(save_right)
	{
		cv::imwrite(filename+std::to_string(filename_num)+"R.png", right_reduced_cv_ptr->image);
		save_right = false;
	}
}


void pdt_init()
{
	save_left = false;
	save_right = false;
	filename = "/home/artezanz/Pictures";
	filename_num = 1;
	_nh = new ros::NodeHandle("~");
	_it = new image_transport::ImageTransport(*_nh);
	left_input_subscriber  = _it->subscribe( "input_left_image_topic",1, &left_image_callback );
	right_input_subscriber = _it->subscribe("input_right_image_topic",1, &right_image_callback);
	left_output_publisher  = _it->advertise( "output_left_image_topic",1);
	right_output_publisher = _it->advertise("output_right_image_topic",1);
	cv::namedWindow("Output_Left",CV_WINDOW_AUTOSIZE); //Test code
	cv::namedWindow("Output_Right",CV_WINDOW_AUTOSIZE); //Test code
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "video_input_node");
	ROS_DEBUG("video_input_node Starting up");

	pdt_init();

	ros::Rate rate(13);

	// ros::NodeHandle _nh("~");
	// image_transport::ImageTransport _it(_nh);

	// VideoInputPrimitive video_input_object; //Creates publishers and subscribers

	ROS_DEBUG("video_input_node Initialized");

	while(ros::ok())
	{
		ros::spinOnce();
		if(cv::waitKey(33)==27)
		{
			save_left = true;
			save_right = true;
		}
	}

	// ros::spin();
	return 0;
}