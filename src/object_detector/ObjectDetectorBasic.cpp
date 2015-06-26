#define MONOCULAR_OBJECTS_DETECTION_LIB //This class is only for monocular detection

#include "ObjectDetectorBasic.hpp"

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <cstdlib>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <string>

#include <boost/gil/gil_all.hpp>

#include "pdt_360deg_git/src/helpers/get_option_value.hpp"
#include "pdt_360deg_git/src/helpers/Log.hpp"

#include "pdt_360deg_git/src/video_input/preprocessing/AddBorderFunctor.hpp"

#include "pdt_360deg_git/src/objects_detection/AbstractObjectsDetector.hpp"
#include "pdt_360deg_git/src/objects_detection/ObjectsDetectorFactory.hpp"

#if !defined(MONOCULAR_OBJECTS_DETECTION_LIB) //Stereo detection libraries, not used here
#include "pdt_360deg_git/src/stereo_matching/AbstractStereoMatcher.hpp"
#include "pdt_360deg_git/src/stereo_matching/cost_volume/DisparityCostVolumeEstimatorFactory.hpp"
#include "pdt_360deg_git/src/stereo_matching/stixels/StixelWorldEstimatorFactory.hpp"
#endif

#include <opencv2/opencv.hpp>  // OpenCV functions, only used for cv::waitKey() //DEBUG

// #include "BasicSdlGui.hpp"

namespace pdt_module
{

using namespace doppia;
using namespace std;

ObjectDetectorBasic::ObjectDetectorBasic(int argc, char** argv) : _nh("~")
{
	next_frame_client = _nh.serviceClient<std_srvs::Empty>("next_frame_service");
    fetch_stereo_client = _nh.serviceClient<pdt_module::FetchStereoImages>("fetch_stereo_service");
    gui_p.reset(new BasicSdlGui());

    boost::program_options::options_description desc("Allowed options");
	get_options_description(desc);
	options = parse_arguments(argc, argv, desc);
	//const bool use_ground_plane = false, use_stixels = false; //!MONOCULAR_OBJECTS_DETECTION_LIB
	init_objects_detection();
}

ObjectDetectorBasic::~ObjectDetectorBasic()
{
	objects_detector_p.reset();
    return;
}

void ObjectDetectorBasic::main_loop()
{
	while(ros::ok())
    {
        //Call service for next frame
        std_srvs::Empty empty_srv;
        next_frame_client.call(empty_srv);

        ROS_INFO("Requested next frame load"); //DEBUG

        //Call service to fetch next frame
        pdt_module::FetchStereoImages stereo_srv;
        while( !(fetch_stereo_client.call(stereo_srv)) )
        {
            ros::Duration(0.002).sleep(); //Sleep for 2 millisecond
        }

        ROS_INFO("Stereo frame loaded. Beginning processing."); //DEBUG

        convert_sensor_to_gil(stereo_srv.response.left_image,  left_image);
        convert_sensor_to_gil(stereo_srv.response.right_image, right_image);

        input_image_const_view_t temp_left = left_image;
        set_monocular_image(temp_left);

        objects_detector_p->compute();

        gui_p->set_stereo_output(left_image, right_image);
        gui_p->update_gui();     
    }
}

//Uses the boost::program_options library to get the configuration settings
boost::program_options::variables_map ObjectDetectorBasic::parse_arguments(int argc, char *argv[], boost::program_options::options_description& desc) const
{
	// boost::program_options::options_description desc("Allowed options");
	// get_options_description(desc);

	//Parse command line arguments command line options
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
        //cout << "\033[1;31mError parsing the command line options:\033[0m " << e.what () << endl << endl;
        //cout << desc << endl;
        ROS_ERROR("Error parsing command line options: %s", e.what());
        exit(EXIT_FAILURE);
    }

    if (options.count("help"))
    {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }

    // parse the configuration file
    {
        std::string configuration_filename;
        if(options.count("configuration_file") > 0)
        {
            configuration_filename = get_option_value<std::string>(options, "configuration_file");
        }
        else
        {
            //cout << "No configuration file provided. Using command line options only." << std::endl;
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

void ObjectDetectorBasic::get_options_description(boost::program_options::options_description &desc) const
{
    desc.add_options()("help", "produces this help message");
	desc.add_options()("configuration_file,c",
						boost::program_options::value<std::string>()->default_value("test_monocular_objects_detection_lib.config.ini"),
						"indicates the path of the configuration .ini file");
	desc.add_options()("save_detections",
						boost::program_options::value<bool>()->default_value(false),
						"save the detected objects in a data sequence file");
    desc.add_options()
            ("video_input.additional_border",
             boost::program_options::value<int>()->default_value(0),
             "when using process_folder, will add border to the image to enable detection of cropped pedestrians. "
             "Value is in pixels (e.g. 50 pixels)");

    // Objects detection options --
    desc.add(ObjectsDetectorFactory::get_args_options());

	#if defined(MONOCULAR_OBJECTS_DETECTION_LIB)
    	// desc.add(AbstractVideoInput::get_args_options());
	#else // not defined(MONOCULAR_OBJECTS_DETECTION_LIB)
	    // we allow input file options, even if we do not use them
	    // desc.add(VideoInputFactory::get_args_options());
	    // Stixel world estimation options --
	    desc.add(AbstractStereoMatcher::get_args_options());
	    desc.add(DisparityCostVolumeEstimatorFactory::get_args_options());
	    desc.add(StixelWorldEstimatorFactory::get_args_options());
	#endif // MONOCULAR_OBJECTS_DETECTION_LIB is defined or not
    return;
}

//Initializes add_border_p, object_detector_p and the logging
void ObjectDetectorBasic::init_objects_detection()
{
	//Initializer for MONOCULAR_OBJECTS_DETECTION_LIB
	// setup the logging //TODO: Needs to be fixed
    {
        logging::get_log().clear(); // we reset previously existing options

        // set our own stdout rules and set cout as console stream --
        logging::LogRuleSet rules_for_stdout;
        rules_for_stdout.add_rule(logging::ErrorMessage, "*"); // we only print errors

        rules_for_stdout.add_rule(logging::WarningMessage, "*"); // also print warnings

        logging::get_log().set_console_stream(std::cout, rules_for_stdout); //TODO: Redirect this to a different stream from std::cout?

        //logging::log(logging::ErrorMessage, "objects_detection") << "Test error message" << std::endl;
    }

    const int additional_border = get_option_value<int>(options, "video_input.additional_border");
    if(additional_border > 0)
    {
        add_border_p.reset(new AddBorderFunctor(additional_border));
    }

    objects_detector_p.reset(ObjectsDetectorFactory::new_instance(options));
}

void ObjectDetectorBasic::set_monocular_image(input_image_const_view_t &input_view)
{
    if(add_border_p)
    {
        //Add the borders for the Markus and Angelos algorithm
        input_image_const_view_t the_input_view = (*add_border_p)(input_view);
        input_dimensions = the_input_view.dimensions();
        objects_detector_p->set_image(the_input_view);
    }
    else
    {
        input_dimensions = input_view.dimensions();
        objects_detector_p->set_image(input_view);
    }
}

void ObjectDetectorBasic::convert_sensor_to_gil(const sensor_msgs::Image& src, boost::gil::rgb8_view_t& dst)
{
    //Ideally a copy-less conversion that redirects the pointer
    dst = boost::gil::interleaved_view(src.width, src.height, (boost::gil::rgb8_pixel_t*)src.data.data(), src.width*3*sizeof(uint8_t));
}

} //exit pdt_module namespace