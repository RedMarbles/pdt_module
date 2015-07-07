

#include "VideoInputSynchronized.hpp"

// #define TEST_COMPILE //Comment this if you don't want the test sections of the code to compile

// #include <cstdlib>

#include <stdexcept>

#include <boost/bind.hpp>

namespace pdt_module
{

// void debug_left(int R, int G, int B)
// {
// 	cv::Mat output_image(300,300,CV_8UC3,cv::Scalar(B,G,R));
// 	cv::imshow("Output_Left",output_image);
// 	cv::waitKey();
// }

// void debug_right(int R, int G, int B)
// {
// 	cv::Mat output_image(300,300,CV_8UC3,cv::Scalar(B,G,R));
// 	cv::imshow("Output_Right",output_image);
// 	cv::waitKey();
// }

VideoInputSynchronized::VideoInputSynchronized(const bool use_gui_) : 
	_nh("~"), 
	left_input_subscriber( _nh, "input_left_image_topic",  1),
	right_input_subscriber(_nh, "input_right_image_topic", 1),
	synced_subscriber(left_input_subscriber, right_input_subscriber, 5),
	use_gui(use_gui_)
{
	/// Tell the video_input_node to load the first frame without being told, so that an image is available to fetch as soon as the stereo topic is available
	flag_load_next_frame = true;

	/// Set up the different preprocessors
	std::string param_cameraType;
	double param_resample_ratio;
	_nh.param("camera_type",param_cameraType,std::string("none"));
	_nh.param("resample_ratio",param_resample_ratio,1.0);

	if(param_cameraType.compare("mei") == 0)
	{
		distort_preprocessor_p.reset(new pdt_module::MeiModelPreprocessor(_nh));
	}
	else if(param_cameraType.compare("none") == 0)
	{
		distort_preprocessor_p.reset(new pdt_module::EmptyPreprocessor(_nh));
	}
	else
	{
		ROS_FATAL("camera_type parameter does not match any known camera model");
		throw std::invalid_argument("camera_type parameter does not match any known camera model");
	}

	if(param_resample_ratio > 1.0)
	{
		resample_preprocessor_p.reset(new pdt_module::ResamplePreprocessor(_nh, param_resample_ratio, CV_8UC3));
	}
	else
	{
		resample_preprocessor_p.reset(new pdt_module::EmptyPreprocessor(_nh));
	}
	
	/// Register the callback for the synchronized subscriber
	synced_subscriber.registerCallback( boost::bind(&VideoInputSynchronized::stereo_image_callback, this, _1, _2) );

	/// Advertise the services for loading and fetching the next frame
	load_next_frame_service =  _nh.advertiseService("load_next_frame_service",  &VideoInputSynchronized::load_next_frame,  this);
	fetch_stereo_frame_service = _nh.advertiseService("fetch_stereo_frame_service", &VideoInputSynchronized::fetch_stereo_frame, this);

	if(use_gui)
	{
		cv::namedWindow("Output_Left",CV_WINDOW_AUTOSIZE);
		cv::namedWindow("Output_Right",CV_WINDOW_AUTOSIZE);
	}

	ROS_INFO("VideoInputSynchronized constructed");
}

VideoInputSynchronized::~VideoInputSynchronized()
{
	ROS_INFO("Destroying VideoInputSynchronized instance"); ///DEBUG
	if(use_gui)
	{
		cv::destroyWindow("Output_Left");
		cv::destroyWindow("Output_Right");
	}
}

void VideoInputSynchronized::stereo_image_callback(const sensor_msgs::ImageConstPtr& input_left_image, const sensor_msgs::ImageConstPtr& input_right_image)
{
	// ROS_INFO("stereo_image_callback called"); ///DEBUG
	if(flag_load_next_frame)
	{
		// ROS_INFO("Loading next frame"); ///DEBUG

		flag_load_next_frame = false;

		///This step is for preprocessing the images. Right now, it undistorts and resamples the image.
		cv_bridge::CvImageConstPtr left_image_cv_ptr;
		cv_bridge::CvImageConstPtr right_image_cv_ptr;
		cv_bridge::CvImagePtr left_rectified_cv_ptr(new cv_bridge::CvImage());
		cv_bridge::CvImagePtr right_rectified_cv_ptr(new cv_bridge::CvImage());
		cv_bridge::CvImagePtr left_reduced_cv_ptr(new cv_bridge::CvImage());
		cv_bridge::CvImagePtr right_reduced_cv_ptr(new cv_bridge::CvImage());
		//sensor_msgs::ImageConstPtr output_left_image; 
		try
		{
			left_image_cv_ptr = cv_bridge::toCvShare(input_left_image,"rgb8");
			right_image_cv_ptr = cv_bridge::toCvShare(input_right_image,"rgb8");
		}
		catch(cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		/// Undistort the fisheye image
		distort_preprocessor_p->processStereoImage(left_image_cv_ptr->image, 
			right_image_cv_ptr->image, 
			left_rectified_cv_ptr->image, 
			right_rectified_cv_ptr->image);

		//Resample the image to half its original size
		resample_preprocessor_p->processStereoImage(left_rectified_cv_ptr->image,
			right_rectified_cv_ptr->image,
			left_reduced_cv_ptr->image,
			right_reduced_cv_ptr->image);

		left_reduced_cv_ptr->encoding  =  left_image_cv_ptr->encoding;
		right_reduced_cv_ptr->encoding = right_image_cv_ptr->encoding;
		left_reduced_cv_ptr->header  =  left_image_cv_ptr->header;
		right_reduced_cv_ptr->header = right_image_cv_ptr->header;

		///Convert image back to a sensor_msgs::Image topic
		left_image_p = left_reduced_cv_ptr->toImageMsg();
		right_image_p = right_reduced_cv_ptr->toImageMsg();

		ROS_INFO("Next frame loaded"); ///DEBUG

		if(use_gui)
		{
			cv::imshow("Output_Left", left_reduced_cv_ptr->image);
			cv::imshow("Output_Right", right_reduced_cv_ptr->image);
			cv::waitKey(1);
		}
	}

	///left_output_publisher.publish( left_reduced_cv_ptr->toImageMsg() );
}

bool VideoInputSynchronized::load_next_frame(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("load_next_frame_service called"); ///DEBUG
	flag_load_next_frame = true;
	return true;
}

bool VideoInputSynchronized::fetch_stereo_frame(pdt_module::FetchStereoImages::Request& req, pdt_module::FetchStereoImages::Response& res)
{
	ROS_INFO("fetch_next_frame_service called"); ///DEBUG
	if(flag_load_next_frame) ///There is still a pending load_next_frame request
	{
		res.left_image.encoding = "Empty";
		res.right_image.encoding = "Empty";
		return false;
	}
	else ///Next image has successfully been loaded
	{
		res.left_image = *left_image_p;
		res.right_image = *right_image_p;
		return true;
	}
}


} ///exit pdt_module namespace
