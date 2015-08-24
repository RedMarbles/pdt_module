
#ifndef _STIXELSESTIMATORBASIC_HPP_
#define _STIXELSESTIMATORBASIC_HPP_

/// ROS includes
#include "ros/ros.h"

/// ROS messages
#include "sensor_msgs/Image.h"
#include "pdt_module/StereoImage.h"
#include "pdt_module/Stixels_msg.h"

/// GUI classes
#include "../AbstractGui.hpp"
#include "../EmptyGui.hpp"
#include "../BasicSdlGui.hpp"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

#include <boost/gil/gil_all.hpp>

#include "pdt_360deg_git/src/video_input/preprocessing/AddBorderFunctor.hpp"

#include "pdt_360deg_git/src/stereo_matching/stixels/Stixel.hpp"
#include "pdt_360deg_git/src/stereo_matching/ground_plane/GroundPlane.hpp"
#include "pdt_360deg_git/src/video_input/MetricStereoCamera.hpp"
#include "pdt_360deg_git/src/video_input/calibration/StereoCameraCalibration.hpp"
#include "pdt_360deg_git/src/stereo_matching/stixels/AbstractStixelWorldEstimator.hpp"

namespace pdt_module
{

class StixelsEstimatorBasic
{
public:
	typedef boost::gil::rgb8_view_t input_image_view_t;
	typedef boost::gil::rgb8c_view_t input_image_const_view_t;
	// typedef doppia::GroundPlane ground_plane_t; /// Unused?
	typedef doppia::stixels_t stixels_t;
	typedef doppia::AbstractStixelWorldEstimator::ground_plane_corridor_t ground_plane_corridor_t; /// Note: actually of std::vector<int> type

protected:
	ros::NodeHandle nh_;
	ros::Subscriber stereo_subscriber;
	ros::Publisher stixels_publisher;

	/// GUI Output
	const bool flag_use_gui;
	boost::scoped_ptr<pdt_module::AbstractGui> gui_p;

	/// Timing variables
	const bool flag_measure_time;
	ros::Time time_begin, time_end;
	ros::Duration duration_processing;

	boost::program_options::variables_map options;

	input_image_view_t left_view;
	input_image_view_t right_view;
	boost::gil::rgb8_image_t::point_t input_dimensions;

	boost::scoped_ptr<doppia::AddBorderFunctor> add_border_p;

	boost::shared_ptr<doppia::AbstractStixelWorldEstimator> stixel_world_estimator_p;

	///Information for initializing stixel_world_estimator_p
	double ground_plane_prior_pitch; /// [radians]
	double ground_plane_prior_roll; /// [radians]
	double ground_plane_prior_height; /// [meters]
	//boost::filesystem::path camera_calib_filename;
	std::string camera_calib_filename;
	boost::shared_ptr<doppia::StereoCameraCalibration> stereo_calibration_p;
	boost::shared_ptr<doppia::MetricStereoCamera> stereo_camera_p;

	stixels_t stixels_extracted;
	ground_plane_corridor_t ground_corridor_extracted;
	

public:
	StixelsEstimatorBasic(int argc, char** argv, ros::NodeHandle& nh__, const bool use_gui = false, const bool measure_time = false);

	~StixelsEstimatorBasic();

protected:
	boost::program_options::variables_map parse_arguments(int argc, char* argv[], boost::program_options::options_description& desc) const;

	void get_options_description(boost::program_options::options_description &desc) const;

	void init_stixels_estimation();

	void stereo_callback(const pdt_module::StereoImage& stereo_message);

	void convert_sensor_to_gil(const sensor_msgs::Image& src, boost::gil::rgb8_view_t& dst) const;

	void set_rectified_stereo_images_pair();

	void compute_stixels();

	void convert_and_publish_stixels();

	void draw_stixels_estimation();

	//void draw_stixels(stixels_t& stixels_local, input_image_view_t& frame_view);
};

} /// Closes pdt_module namespace

#endif 
///endblock _STIXELSESTIMATORBASIC_HPP_