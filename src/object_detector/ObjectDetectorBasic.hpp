
#ifndef _OBJECTDETECTORBASIC_HPP_
#define _OBJECTDETECTORBASIC_HPP_

#include "ros/ros.h"

#include "std_srvs/Empty.h"
#include "pdt_module/FetchStereoImages.h"

#include "BasicSdlGui.hpp"

#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

#include "pdt_360deg_git/src/video_input/preprocessing/AddBorderFunctor.hpp"

#include "pdt_360deg_git/src/objects_detection/AbstractObjectsDetector.hpp"

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
private:
	ros::NodeHandle _nh;
	ros::ServiceClient next_frame_client;
	ros::ServiceClient fetch_stereo_client;

	boost::scoped_ptr<pdt_module::BasicSdlGui> gui_p;

	boost::program_options::variables_map options;

	boost::scoped_ptr<doppia::AddBorderFunctor> add_border_p;

	boost::shared_ptr<doppia::AbstractObjectsDetector> objects_detector_p;

	doppia::AbstractObjectsDetector::detections_t detections; //A std::vector<Detections2D> type object

	// boost::gil::rgb8_view_t left_image;  //input_image_view_t
	// boost::gil::rgb8_view_t right_image; //input_image_view_t
	input_image_view_t left_image;  //input_image_view_t
	input_image_view_t right_image; //input_image_view_t
	boost::gil::rgb8_image_t::point_t input_dimensions;

public:

	ObjectDetectorBasic(int argc, char** argv);

	~ObjectDetectorBasic();

	void main_loop();

	boost::program_options::variables_map parse_arguments(int argc, char *argv[], boost::program_options::options_description& desc) const;

	void get_options_description(boost::program_options::options_description &desc) const;

	void init_objects_detection();

	void set_monocular_image(input_image_const_view_t &input_view);

	void convert_sensor_to_gil(const sensor_msgs::Image& src, boost::gil::rgb8_view_t& dst);

	void draw_detections(detections_t& detections_local, input_image_view_t& frame_view);
};

} // exit pdt_module namespace


#endif
//endblock _OBJECTDETECTORBASIC_HPP_