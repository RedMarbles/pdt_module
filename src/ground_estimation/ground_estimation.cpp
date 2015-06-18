//This implements a ROS-compatible version of the ground_estimation doppia application

#include "ros/ros.h"

#include <cstdlib>
#include <iostream>

#include <boost/scoped_ptr.hpp>

#include "libs/pdt_360deg_git/src/applications/ground_estimation/GroundEstimationApplication.hpp"

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ground_estimation_basic_node");
	ros::NodeHandle _nh("~");

	int ret = EXIT_SUCCESS;

	//std::string ConfigFileName;
	//_nh.param<std::string>("ConfigFile",ConfigFileName,"default_filename");
	//cout<<"Filename received: "<<ConfigFileName<<endl;

	try
	{
		boost::scoped_ptr<doppia::AbstractApplication>
			application_p( new doppia::GroundEstimationApplication() );

		ret = application_p->main(argc, argv);
	}
	catch (std::exception &e)
	{
		cout<<"\033[1;31mA std::exception was raised:\033[0m " << e.what() << endl;
		ret = EXIT_FAILURE;
		throw;
	}
	catch (...)
	{
		cout<<"\033[1;31mAn unknown exception was raised\033[0m"<<endl;
		ret = EXIT_FAILURE;
		throw;
	}

	return ret;
}