
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
private:
	ros::NodeHandle _nh;
	ros::ServiceClient next_frame_client;
	ros::ServiceClient fetch_stereo_client;

	scoped_ptr<BasicSdlGui> gui_p;

	boost::program_options::variables_map options;

	scoped_ptr<AddBorderFunctor> add_border_p;

	shared_ptr<AbstractObjectsDetector> objects_detector_p;

	boost::gil::rgb8_pixel_t left_image;  //input_image_view_t
	boost::gil::rgb8_pixel_t right_image; //input_image_view_t
	boost::gil::rgb8_image_t::point_t input_dimensions;

public:
	typedef boost::gil::rgb8_view_t input_image_view_t;
	typedef boost::gil::rgb8c_view_t input_image_const_view_t;

	ObjectDetectorBasic(int argc, char** argv);

	~ObjectDetectorBasic();

	void main_loop();

	boost::program_options::variables_map& parse_arguments(int argc, char *argv[]) const;

	void get_options_description(boost::program_options::options_description &desc) const;

	void init_objects_detection();

	void set_monocular_image(input_image_const_view_t &input_view);

	void convert_sensor_to_gil(const sensor_msgs::Image& src, boost::gil::rgb8_view_t& dst);
};

} // exit pdt_module namespace


#endif
//endblock _OBJECTDETECTORBASIC_HPP_