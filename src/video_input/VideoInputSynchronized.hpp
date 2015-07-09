
#ifndef _VIDEOINPUTSYNCHRONIZED_HPP_
#define _VIDEOINPUTSYNCHRONIZED_HPP_

/// ROS includes
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
// #include "pdt_module/FetchStereoImages.h"
#include "pdt_module/StereoImage.h"
// #include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

/// Preprocessor classes
#include "preprocessor/AbstractPreprocessor.hpp"
#include "preprocessor/MeiModelPreprocessor.hpp"
#include "preprocessor/ResamplePreprocessor.hpp"
#include "preprocessor/EmptyPreprocessor.hpp"

/// OpenCV library (for image handling)
#include <opencv2/opencv.hpp> 

// /// Boost GIL library (for image handling)
// #include <boost/gil/gil_all.hpp>

#include <boost/shared_ptr.hpp>

namespace pdt_module
{

class VideoInputSynchronized
{
private:
	ros::NodeHandle nh_;

	/// ROS Subscribers
	message_filters::Subscriber<sensor_msgs::Image> left_input_subscriber;
	message_filters::Subscriber<sensor_msgs::Image> right_input_subscriber;
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> synced_subscriber;

	/// ROS Publishers
	ros::Publisher stereo_image_publisher;

	// /// ROS Services
	// ros::ServiceServer load_next_frame_service;
	// ros::ServiceServer fetch_stereo_frame_service;
	// sensor_msgs::ImagePtr left_image_p;
	// sensor_msgs::ImagePtr right_image_p;
	// bool flag_load_next_frame;

	/// Preprocessor objects
	boost::shared_ptr<pdt_module::AbstractPreprocessor> distort_preprocessor_p;
	boost::shared_ptr<pdt_module::AbstractPreprocessor> resample_preprocessor_p;

	/// Flags to trigger debugging options
	const bool use_gui; /// If 'true', outputs the processed frames 
	const bool measure_time; /// If 'true', outputs the time required for processing
	ros::Time time_begin, time_end;
	ros::Duration duration_processing;

public:
	VideoInputSynchronized(ros::NodeHandle& nh__, const bool use_gui_ = false, const bool measure_time_ = false);

	~VideoInputSynchronized();

	void stereo_image_callback(const sensor_msgs::ImageConstPtr& input_left_image, const sensor_msgs::ImageConstPtr& input_right_image);

	// bool load_next_frame(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	// bool fetch_stereo_frame(pdt_module::FetchStereoImages::Request& req, pdt_module::FetchStereoImages::Response& res);
};

} //exit pdt_module namespace

#endif 
//endblock _VIDEOINPUTSYNCHRONIZED_HPP_