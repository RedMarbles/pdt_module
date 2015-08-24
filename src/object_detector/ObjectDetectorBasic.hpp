
#ifndef _OBJECTDETECTORBASIC_HPP_
#define _OBJECTDETECTORBASIC_HPP_

/// ROS files
#include "ros/ros.h"

/// ROS services
#include "std_srvs/Empty.h"
#include "pdt_module/FetchStereoImages.h"

/// ROS messages
#include "sensor_msgs/Image.h"
#include "pdt_module/StereoImage.h"

/// GUI classes
#include "../AbstractGui.hpp"
#include "../EmptyGui.hpp"
#include "../BasicSdlGui.hpp"

#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

#include <boost/gil/gil_all.hpp>

#include "pdt_360deg_git/src/video_input/preprocessing/AddBorderFunctor.hpp"

#include "pdt_360deg_git/src/objects_detection/AbstractObjectsDetector.hpp"
#include "pdt_360deg_git/src/objects_detection/Detection2d.hpp"

//#include "pdt_360deg_git/src/applications/objects_detection_lib/objects_detection_lib.hpp" //Load pre-built object detection library

namespace pdt_module
{

class ObjectDetectorBasic
{
public:
	typedef boost::gil::rgb8_view_t input_image_view_t;
	typedef boost::gil::rgb8c_view_t input_image_const_view_t;
	typedef doppia::AbstractObjectsDetector::detection_t detection_t;
	typedef doppia::AbstractObjectsDetector::detections_t detections_t;

protected:
	ros::NodeHandle nh_;
	// ros::ServiceClient next_frame_client;
	// ros::ServiceClient fetch_stereo_client;
	ros::Subscriber stereo_subscriber;

	/// GUI Output
	const bool flag_use_gui;
	boost::scoped_ptr<pdt_module::AbstractGui> gui_p;

	boost::program_options::variables_map options;

	boost::scoped_ptr<doppia::AddBorderFunctor> add_border_p;

	boost::shared_ptr<doppia::AbstractObjectsDetector> objects_detector_p;

	doppia::AbstractObjectsDetector::detections_t detections; //A std::vector<Detections2D> type object

	// boost::gil::rgb8_view_t left_image;  //input_image_view_t
	// boost::gil::rgb8_view_t right_image; //input_image_view_t
	input_image_view_t left_image;  //input_image_view_t
	input_image_view_t right_image; //input_image_view_t
	boost::gil::rgb8_image_t::point_t input_dimensions;

	/// Timing variables
	const bool flag_measure_time;
	ros::Time time_begin, time_end;
	ros::Duration duration_processing;

public:

	ObjectDetectorBasic(int argc, char** argv, ros::NodeHandle& nh__, const bool use_gui_ = false, const bool measure_time_ = false);

	~ObjectDetectorBasic();

	// void main_loop();

protected:

	boost::program_options::variables_map parse_arguments(int argc, char *argv[], boost::program_options::options_description& desc) const;

	void get_options_description(boost::program_options::options_description &desc) const;

	void init_objects_detection();

	void set_monocular_image(input_image_const_view_t &input_view);

	void stereo_monocular_callback(const pdt_module::StereoImage& stereo_message);

	void convert_sensor_to_gil(const sensor_msgs::Image& src, boost::gil::rgb8_view_t& dst);

	void draw_detections(detections_t& detections_local, input_image_view_t& frame_view);
};

} // exit pdt_module namespace


#endif
//endblock _OBJECTDETECTORBASIC_HPP_