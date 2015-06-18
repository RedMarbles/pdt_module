

#include "VideoInputPrimitive.hpp"

#define TEST_COMPILE //Comment this if you don't want the test sections of the code to compile

// #include <cstdlib>

// #include <boost/bind.hpp>

namespace pdt_module
{

void debug_left(int R, int G, int B)
{
	cv::Mat output_image(300,300,CV_8UC3,cv::Scalar(B,G,R));
	cv::imshow("Output_Left",output_image);
	cv::waitKey();
}

void debug_right(int R, int G, int B)
{
	cv::Mat output_image(300,300,CV_8UC3,cv::Scalar(B,G,R));
	cv::imshow("Output_Right",output_image);
	cv::waitKey();
}

VideoInputPrimitive::VideoInputPrimitive() : _nh("~"), _it(_nh)
{
	left_input_subscriber  = _it.subscribe("input_left_image_topic" , 1,  &VideoInputPrimitive::left_image_callback, this);
	right_input_subscriber = _it.subscribe("input_right_image_topic", 1, &VideoInputPrimitive::right_image_callback, this);
	left_output_publisher  = _it.advertise( "output_left_image_topic",1);
	right_output_publisher = _it.advertise("output_right_image_topic",1);
#ifdef TEST_COMPILE
	cv::namedWindow("Output_Left",CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Output_Right",CV_WINDOW_AUTOSIZE);
#endif
	ROS_INFO("VideoInputPrimitive constructed");
}

VideoInputPrimitive::~VideoInputPrimitive()
{
	//Do Nothing
	ROS_INFO("Destroying VideoInputPrimitive instance");
#ifdef TEST_COMPILE
	cv::destroyWindow("Output_Left");
	cv::destroyWindow("Output_Right");
#endif
}

void VideoInputPrimitive::left_image_callback(const sensor_msgs::ImageConstPtr& input_left_image)
{
	ROS_INFO("left_image_callback called");
	cv_bridge::CvImageConstPtr left_image_cv_ptr;
	cv_bridge::CvImagePtr left_reduced_cv_ptr(new cv_bridge::CvImage());
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
	catch(...)
	{
		ROS_ERROR("Unknown exception in left_image_callback");
		return;
	}
	const int orig_rows = left_image_cv_ptr->image.rows;
	const int orig_cols = left_image_cv_ptr->image.cols;
	left_reduced_cv_ptr->image.create( cv::Size( orig_cols/2, orig_rows/2 ), CV_8UC3 );

	cv::pyrDown( left_image_cv_ptr->image, left_reduced_cv_ptr->image, cv::Size( orig_cols/2, orig_rows/2 ));
	left_reduced_cv_ptr->encoding = left_image_cv_ptr->encoding;
	left_reduced_cv_ptr->header = left_image_cv_ptr->header;

#ifdef TEST_COMPILE
	cv::imshow("Output_Left", left_reduced_cv_ptr->image);
	cv::waitKey(1);
#endif

	left_output_publisher.publish( left_reduced_cv_ptr->toImageMsg() );
}


void VideoInputPrimitive::right_image_callback(const sensor_msgs::ImageConstPtr& input_right_image)
{
	ROS_INFO("right_image_callback called");
	cv_bridge::CvImageConstPtr right_image_cv_ptr;
	cv_bridge::CvImagePtr right_reduced_cv_ptr(new cv_bridge::CvImage());
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
	catch(...)
	{
		ROS_ERROR("Unknown exception in right_image_callback");
		return;
	}
	
	const int orig_rows = right_image_cv_ptr->image.rows;
	const int orig_cols = right_image_cv_ptr->image.cols;
	right_reduced_cv_ptr->image.create( cv::Size( orig_cols/2, orig_rows/2 ), CV_8UC3 );

	cv::pyrDown( right_image_cv_ptr->image, right_reduced_cv_ptr->image, cv::Size( orig_cols/2, orig_rows/2 ));
	right_reduced_cv_ptr->encoding = right_image_cv_ptr->encoding;
	right_reduced_cv_ptr->header = right_image_cv_ptr->header;

#ifdef TEST_COMPILE
	cv::imshow("Output_Right", right_reduced_cv_ptr->image);
	cv::waitKey(1);
#endif

	right_output_publisher.publish( right_reduced_cv_ptr->toImageMsg() );
}


} //exit pdt_module namespace
