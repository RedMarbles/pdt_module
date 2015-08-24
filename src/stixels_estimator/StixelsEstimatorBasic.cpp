
#include "StixelsEstimatorBasic.hpp"

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <cstdlib>

// #include <boost/filesystem.hpp>
// #include <boost/program_options.hpp>
// #include <boost/shared_ptr.hpp>
// #include <boost/scoped_ptr.hpp>
#include <string>

// #include <boost/gil/gil_all.hpp>

/// Miscellaneous helper functions
#include "pdt_360deg_git/src/helpers/get_option_value.hpp"
#include "pdt_360deg_git/src/helpers/Log.hpp"

// #include "pdt_360deg_git/src/video_input/VideoInputFactory.hpp"
#include "pdt_360deg_git/src/video_input/calibration/StereoCameraCalibration.hpp"
#include "pdt_360deg_git/src/stereo_matching/AbstractStereoMatcher.hpp"
#include "pdt_360deg_git/src/stereo_matching/cost_volume/DisparityCostVolumeEstimatorFactory.hpp"
#include "pdt_360deg_git/src/stereo_matching/stixels/StixelWorldEstimatorFactory.hpp"

/// Drawing function helpers
// #include <boost/foreach.hpp>
// #include "pdt_360deg_git/src/drawing/gil/colors.hpp"
// #include "pdt_360deg_git/src/drawing/gil/line.hpp"
#include "pdt_360deg_git/src/stereo_matching/stixels/StixelWorldEstimator.hpp"
#include "pdt_360deg_git/src/stereo_matching/stixels/FastStixelWorldEstimator.hpp"
#include "pdt_360deg_git/src/stereo_matching/stixels/FastStixelsEstimator.hpp"
#include "pdt_360deg_git/src/stereo_matching/stixels/ImagePlaneStixelsEstimator.hpp"
#include "pdt_360deg_git/src/drawing/gil/draw_stixel_world.hpp"


namespace pdt_module
{

using namespace doppia;
using namespace std;

StixelsEstimatorBasic::StixelsEstimatorBasic(int argc, char** argv, ros::NodeHandle& nh__, const bool use_gui_, const bool measure_time_) :
	nh_(nh__),
	flag_use_gui(use_gui_),
	flag_measure_time(measure_time_)
{
	stereo_subscriber = nh_.subscribe("RectifiedStereo", 1, &StixelsEstimatorBasic::stereo_callback, this);

	if(flag_use_gui)
	{
		gui_p.reset(new BasicSdlGui());
	}
	else
	{
		gui_p.reset(new EmptyGui());
	}

    // std::string camera_calib_filename_string;
    // nh_.getParam("camera_calibration_protobuf",camera_calib_filename_string);
    // camera_calib_filename = camera_calib_filename_string; /// Convert string to filename format
    nh_.getParam("camera_calibration_protobuf",camera_calib_filename);
    stixels_publisher = nh_.advertise<pdt_module::Stixels_msg>("Stixels", 20); /// Arbitary buffer size of 20

	boost::program_options::options_description desc("Allowed options");
	get_options_description(desc);
	options = parse_arguments(argc, argv, desc);
	init_stixels_estimation();

	/// Initialize timing variables
	time_begin = ros::Time::now();
	time_end = ros::Time::now();
	duration_processing = ros::Duration(1.0);

	ROS_INFO("StixelsEstimatorBasic initialization complete");
}

StixelsEstimatorBasic::~StixelsEstimatorBasic()
{
	stixel_world_estimator_p.reset();
	gui_p.reset();
	return;
}

boost::program_options::variables_map StixelsEstimatorBasic::parse_arguments(int argc, char** argv, boost::program_options::options_description& desc) const
{
	/// Parse command line arguments command line options
	boost::program_options::variables_map options;
    try
    {
        boost::program_options::command_line_parser parser(argc, argv);
        parser.options(desc);
        const boost::program_options::parsed_options the_parsed_options( parser.run() );

        boost::program_options::store(the_parsed_options, options);
        //program_options::store(program_options::parse_command_line(argc, argv, desc), options);
        boost::program_options::notify(options);
    }
    catch (std::exception & e)
    {
        // cout << "\033[1;31mError parsing the command line options:\033[0m " << e.what () << endl << endl;
        // cout << desc << endl;
        ROS_ERROR("Error parsing command line options: %s", e.what());
        exit(EXIT_FAILURE);
    }

    if (options.count("help"))
    {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }

    /// Parse the configuration file
    {
        std::string configuration_filename;
        if(options.count("configuration_file") > 0)
        {
            configuration_filename = get_option_value<std::string>(options, "configuration_file");
        }
        else
        {
            // cout << "No configuration file provided. Using command line options only." << std::endl;
            ROS_ERROR("No configuration file provided. Using command line options only.");
        }
        if (configuration_filename.empty() == false)
        {
            boost::filesystem::path configuration_file_path(configuration_filename);
            if(boost::filesystem::exists(configuration_file_path) == false)
            {
                // cout << "\033[1;31mCould not find the configuration file:\033[0m "
                //      << configuration_file_path << endl;
                ROS_ERROR("Could not find the configuration file: %s",configuration_filename.c_str());
                return options;
            }
            ROS_INFO("Going to parse the configuration file: %s\n", configuration_filename.c_str());
            try
            {
                fstream configuration_file;
                configuration_file.open(configuration_filename.c_str(), fstream::in);
                boost::program_options::store(boost::program_options::parse_config_file(configuration_file, desc), options);
                configuration_file.close();
            }
            catch (...)
            {
                // cout << "\033[1;31mError parsing THE configuration file named:\033[0m "
                //      << configuration_filename << endl;
                // cout << desc << endl;
                ROS_ERROR("Error parsing the configuration file named: %s",configuration_filename.c_str());
                throw;
            }
            // cout << "Parsed the configuration file " << configuration_filename << std::endl;
            ROS_INFO("Parsed the configuration file %s", configuration_filename.c_str());
        }
        else
        {
        	ROS_ERROR("Configuration file is empty");
        	exit(EXIT_FAILURE);
        }
    }
    return options;
}

void StixelsEstimatorBasic::get_options_description(boost::program_options::options_description& desc) const
{
	desc.add_options()("help", "produces this help message");
	desc.add_options()("configuration_file,c",
						boost::program_options::value<std::string>()->default_value("./test_objects_detection_lib.config.ini"),
						"indicates the path of the configuration .ini file");
	desc.add_options()("save_detections",
						boost::program_options::value<bool>()->default_value(false),
						"save the detected objects in a data sequence file");
    desc.add_options()
            ("video_input.additional_border",
             boost::program_options::value<int>()->default_value(0),
             "when using process_folder, will add border to the image to enable detection of cropped pedestrians. "
             "Value is in pixels (e.g. 50 pixels)");

	desc.add(AbstractStereoMatcher::get_args_options());
    desc.add(DisparityCostVolumeEstimatorFactory::get_args_options());
    desc.add(StixelWorldEstimatorFactory::get_args_options());
}

void StixelsEstimatorBasic::init_stixels_estimation()
{
	/// setup the logging //TODO: Needs to be fixed
    {
        logging::get_log().clear(); /// we reset previously existing options

        /// set our own stdout rules and set cout as console stream --
        logging::LogRuleSet rules_for_stdout;
        rules_for_stdout.add_rule(logging::ErrorMessage, "*"); /// we only print errors

        rules_for_stdout.add_rule(logging::WarningMessage, "*"); /// also print warnings

        logging::get_log().set_console_stream(std::cout, rules_for_stdout); //TODO: Redirect this to a different stream from std::cout?

        //logging::log(logging::ErrorMessage, "objects_detection") << "Test error message" << std::endl;
    }

    const int additional_border = get_option_value<int>(options, "video_input.additional_border");
    if(additional_border > 0)
    {
        add_border_p.reset(new AddBorderFunctor(additional_border));
    }

    //boost::filesystem::path camera_calib_filename = get_option_value<std::string>(options, "video_input.calibration_filename");
	stereo_calibration_p.reset(new doppia::StereoCameraCalibration(camera_calib_filename));
	stereo_camera_p.reset(new doppia::MetricStereoCamera(*stereo_calibration_p));
	// ground_plane_prior_height = get_option_value<float>(options,"video_input.camera_height");
	// ground_plane_prior_pitch = get_option_value<float>(options,"video_input.camera_pitch");
	// ground_plane_prior_roll = get_option_value<float>(options,"video_input.camera_roll");

    nh_.param("camera_height",ground_plane_prior_height,1.0);
    nh_.param("camera_roll"  ,ground_plane_prior_roll  ,0.0);
    nh_.param("camera_pitch" ,ground_plane_prior_pitch ,0.0);
    /// Since StixelWorldEstimatorFactory needs the image dimensions, stixels_estimator_p is only initialized after the first image is received
}

void StixelsEstimatorBasic::stereo_callback(const pdt_module::StereoImage& stereo_message)
{
    if(flag_measure_time)
    {
        time_begin = ros::Time::now();
    }

    /// Convert input image into GIL format
    convert_sensor_to_gil(stereo_message.left_image,  left_view);
    convert_sensor_to_gil(stereo_message.right_image, right_view);

    set_rectified_stereo_images_pair();
    compute_stixels();
    convert_and_publish_stixels();

    if(flag_use_gui)
    {
    	draw_stixels_estimation();
    }

    if(flag_measure_time)
    {
    	time_end = ros::Time::now();
    	duration_processing = time_end - time_begin;
    	ROS_INFO("Processing time : [%.5f s]  ,  Rate : [%.2f Hz]",duration_processing.toSec(), 1/duration_processing.toSec());
    }
}

void StixelsEstimatorBasic::convert_sensor_to_gil(const sensor_msgs::Image& src, boost::gil::rgb8_view_t& dst) const
{
    /// Ideally a copy-less conversion that redirects the pointer
    dst = boost::gil::interleaved_view(src.width, src.height, (boost::gil::rgb8_pixel_t*)src.data.data(), src.width*3*sizeof(uint8_t));
}

void StixelsEstimatorBasic::set_rectified_stereo_images_pair()
{
	///In case it's the first frame, initialize stixel_world_estimator_p (since it needs the image dimensions, it can't be initialized any earlier)
	if(!stixel_world_estimator_p)
	{
		stixel_world_estimator_p.reset(
                        StixelWorldEstimatorFactory::new_instance(options,
                                                                  left_view.dimensions(),
                                                                  *stereo_camera_p,
                                                                  ground_plane_prior_pitch,
                                                                  ground_plane_prior_roll,
                                                                  ground_plane_prior_height));
	}
    input_image_const_view_t left_view_const  = left_view;
    input_image_const_view_t right_view_const = right_view;
	stixel_world_estimator_p->set_rectified_images_pair(left_view_const, right_view_const);
}

void StixelsEstimatorBasic::compute_stixels()
{
	stixel_world_estimator_p->compute();

	ground_corridor_extracted = stixel_world_estimator_p->get_ground_plane_corridor();
	stixels_extracted = stixel_world_estimator_p->get_stixels();
	// object_detector_p->set_ground_plane_corridor(ground_corridor);
	// object_detector_p->set_stixels(stixels_calculated);
}

void StixelsEstimatorBasic::convert_and_publish_stixels()
{
    pdt_module::Stixels_msg stixels_message;

    stixels_message.ground_corridor = ground_corridor_extracted;

    stixels_message.stixels.resize( stixels_extracted.size() );
    for(size_t i=0; i<stixels_extracted.size(); i++)
    {
        stixels_message.stixels[i].width                  = stixels_extracted[i].width;
        stixels_message.stixels[i].x                      = stixels_extracted[i].x;
        stixels_message.stixels[i].bottom_y               = stixels_extracted[i].bottom_y;
        stixels_message.stixels[i].top_y                  = stixels_extracted[i].top_y;
        stixels_message.stixels[i].default_height_value   = stixels_extracted[i].default_height_value;
        stixels_message.stixels[i].disparity              = stixels_extracted[i].disparity;
        stixels_message.stixels[i].type                   = stixels_extracted[i].type; /// Type is originally an enum, so will need to static_cast<doppia::Stixel::Types> to get it back
        stixels_message.stixels[i].backward_delta_x       = stixels_extracted[i].backward_delta_x;
        stixels_message.stixels[i].valid_backward_delta_x = stixels_extracted[i].valid_backward_delta_x;
        stixels_message.stixels[i].backward_width         = stixels_extracted[i].backward_width;
    }

    stixels_publisher.publish(stixels_message);
}

void StixelsEstimatorBasic::draw_stixels_estimation()
{
    //Create output images
    boost::gil::rgb8_image_t screen_left_image(left_view.dimensions());
    boost::gil::rgb8_image_t screen_right_image(right_view.dimensions());
    boost::gil::rgb8_view_t screen_left_view = boost::gil::view(screen_left_image);
    boost::gil::rgb8_view_t screen_right_view = boost::gil::view(screen_right_image);
    // screen_left_view.recreate(left_view.width(), left_view.height());
    // screen_right_view.recreate(left_view.width(), left_view.height());

    // doppia::StixelWorldEstimator *the_stixel_world_estimator_p = dynamic_cast< doppia::StixelWorldEstimator *>(stixel_world_estimator_p.get());
    // doppia::FastStixelWorldEstimator *the_fast_stixel_world_estimator_p = dynamic_cast< doppia::FastStixelWorldEstimator *>(stixel_world_estimator_p.get());

    // if(the_stixel_world_estimator_p != NULL)
    // {
    //     doppia::draw_stixels_estimation( *(the_stixel_world_estimator_p->stixels_estimator_p),
    //                                     left_view,
    //                                     screen_left_view,
    //                                     screen_right_view);
    // }
    // else if(the_fast_stixel_world_estimator_p != NULL)
    // {
    //     const doppia::FastStixelsEstimator *fast_estimator_p = dynamic_cast< doppia::FastStixelsEstimator *> (the_fast_stixel_world_estimator_p->stixels_estimator_p.get());
    //     const doppia::ImagePlaneStixelsEstimator *uv_estimator_p = dynamic_cast< doppia::ImagePlaneStixelsEstimator *> (the_fast_stixel_world_estimator_p->stixels_estimator_p.get());

    //     if(fast_estimator_p != NULL)
    //     {
    //         doppia::draw_stixels_estimation(*fast_estimator_p, left_view, screen_left_view, screen_right_view);
    //     }
    //     else if(uv_estimator_p != NULL)
    //     {
    //         doppia::draw_stixels_estimation(*uv_estimator_p, left_view, screen_left_view, screen_right_view);
    //     }
    //     else
    //     {
    //         // Do nothing
    //     }
    // }
    // else
    // {
    //     // Do nothing
    // }

    doppia::draw_the_stixels(left_view, stixels_extracted);

    // gui_p->set_stereo_output(screen_left_view, screen_right_view);
    gui_p->set_stereo_output(left_view, right_view);
    gui_p->update_gui();

    return;
}

} /// exit pdt_module namespace