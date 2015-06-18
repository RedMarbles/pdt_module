

#include "VideoInputVeryPrimitive.hpp"

#include <boost/shared_ptr.hpp>


namespace pdt_module
{

#ifdef TEST_COMPILE
void VideoInputVeryPrimitive::sampleOutput()
{
	ROS_INFO("Entering sampleOutput()");
	cv::Mat left_image(400,400,CV_8UC3,cv::Scalar(0,0,255));
	cv::Mat right_image(400,400,CV_8UC3,cv::Scalar(0,255,0));
	std_msgs::Header header;
	cv_bridge::CvImagePtr  left_image_p(new cv_bridge::CvImage(header, "bgr8", left_image));
	cv_bridge::CvImagePtr right_image_p(new cv_bridge::CvImage(header, "bgr8", right_image));
	//Output images to windows
	cv::imshow("Output_Left", left_image );
	cv::imshow("Output_Right", right_image );
	//Publish images
	left_output_publisher.publish( left_image_p->toImageMsg() );
	right_output_publisher.publish( right_image_p->toImageMsg() );
	cv::waitKey();
	ROS_INFO("Exiting sampleOutput()");
}
#endif

VideoInputVeryPrimitive::VideoInputVeryPrimitive() : _nh("~")
{
	left_input_subscriber = _nh.subscribe("input_left_image_topic", 1, &VideoInputVeryPrimitive::left_image_callback, this);
	right_input_subscriber = _nh.subscribe("input_right_image_topic", 1, &VideoInputVeryPrimitive::right_image_callback, this);
	left_output_publisher = _nh.advertise<sensor_msgs::Image>("output_left_image_topic",1);
	right_output_publisher = _nh.advertise<sensor_msgs::Image>("output_right_image_topic",1);
#ifdef TEST_COMPILE
	cv::namedWindow("Output_Left", cv::WINDOW_AUTOSIZE); ROS_INFO("Debug 1");
	cv::namedWindow("Output_Right", cv::WINDOW_AUTOSIZE); ROS_INFO("Debug 2");
	sampleOutput(); ROS_INFO("Debug 3");
#endif
	ROS_INFO("VideoInputVeryPrimitive instance constructed");
}

VideoInputVeryPrimitive::~VideoInputVeryPrimitive()
{
	//Do nothing
#ifdef TEST_COMPILE
	cv::destroyAllWindows();
#endif
}

void VideoInputVeryPrimitive::left_image_callback(const sensor_msgs::ImageConstPtr& input_left_image)
{
	ROS_INFO("left_image_callback called");
	cv_bridge::CvImageConstPtr left_image_cv_ptr;
	cv_bridge::CvImagePtr left_reduced_cv_ptr;
	sensor_msgs::ImageConstPtr output_left_image;
	try
	{
		left_image_cv_ptr = cv_bridge::toCvShare(input_left_image);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	const int orig_rows = left_image_cv_ptr->image.rows;
	const int orig_cols = left_image_cv_ptr->image.cols;

	cv::pyrDown( left_image_cv_ptr->image, left_reduced_cv_ptr->image, cv::Size( orig_cols/2, orig_rows/2 ));
	left_reduced_cv_ptr->encoding = left_image_cv_ptr->encoding;
	left_reduced_cv_ptr->header = left_image_cv_ptr->header;

#ifdef TEST_COMPILE
	cv::imshow("Output_Left", left_reduced_cv_ptr->image); //Test code
	cv::waitKey();
#endif

	left_output_publisher.publish( left_reduced_cv_ptr->toImageMsg() );
}

void VideoInputVeryPrimitive::right_image_callback(const sensor_msgs::ImageConstPtr& input_right_image)
{
	ROS_INFO("right_image_callback called");
	cv_bridge::CvImageConstPtr right_image_cv_ptr;
	cv_bridge::CvImagePtr right_reduced_cv_ptr;
	sensor_msgs::ImageConstPtr output_right_image;
	try
	{
		right_image_cv_ptr = cv_bridge::toCvShare(input_right_image);
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

#ifdef TEST_COMPILE
	cv::imshow("Output_Right", right_reduced_cv_ptr->image); //Test code
	cv::waitKey();
#endif

	right_output_publisher.publish( right_reduced_cv_ptr->toImageMsg() );
}

} //exit pdt_module namespace