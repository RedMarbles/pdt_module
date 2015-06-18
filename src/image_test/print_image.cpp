// Accepts image from input and prints it to screen using SDL

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/Empty.h"

#include <boost/gil/gil_all.hpp>

#include <SDL2/SDL.h>

SDL_Surface *screen_p;
SDL_Window  *window_p;
boost::gil::rgb8_image_t screen_image;
boost::gil::rgb8_view_t screen_image_view;
boost::gil::rgb8_view_t screen_left_view, screen_right_view;

bool gui_initialized;

//Creates a window of the specified size and also initializes the gil image views
void init_gui(const int input_width, const int input_height)
{
	//Initialize the SDL Window interfaces
	SDL_Init(SDL_INIT_VIDEO);
	window_p = SDL_CreateWindow( "Output", 
								SDL_WINDOWPOS_UNDEFINED,
								SDL_WINDOWPOS_UNDEFINED,
								input_width*2,
								input_height,
								SDL_WINDOW_SHOWN);
	screen_p = SDL_GetWindowSurface( window_p );

	//Initialize the global image containers
	screen_image.recreate(input_width*2,input_height);
	screen_image_view = boost::gil::view(screen_image);
	screen_left_view = boost::gil::subimage_view(screen_image_view,0,0,input_width,input_height);
	screen_right_view = boost::gil::subimage_view(screen_image_view,input_width,0,input_width,input_height);
	boost::gil::fill_pixels(screen_right_view,boost::gil::rgb8_pixel_t(0,255,255));

	gui_initialized = true;
}

void update_gui()
{
	const int depth = 24;
    //const int pitch = (view.row_begin(1) - view.row_begin(0)) / sizeof(boost::gil::rgb8c_pixel_t);
    const int pitch = screen_image_view.width()*3;


    // SDL interprets each pixel as a 32-bit number, so our masks must depend
    // on the endianness (byte order) of the machine
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
    //const Uint32  r_mask = 0xff000000, g_mask = 0x00ff0000, b_mask = 0x0000ff00, a_mask = 0x000000ff;
    const Uint32 r_mask = 0x00ff0000, g_mask = 0x0000ff00, b_mask = 0x000000ff, a_mask = 0xff000000;
#else
    //This Little Endian is the form on this system.
    //const Uint32 r_mask = 0x000000ff, g_mask = 0x0000ff00, b_mask = 0x00ff0000, a_mask = 0xff000000;
    const Uint32 r_mask = 0x000000ff, g_mask = 0x0000ff00, b_mask = 0x00ff0000, a_mask = 0xff000000;
#endif

	SDL_Surface *surface_p = SDL_CreateRGBSurfaceFrom( boost::gil::interleaved_view_get_raw_data(screen_image_view),
														screen_image_view.width(),
														screen_image_view.height(),
														depth,
														pitch,
														r_mask, g_mask, b_mask, a_mask);
	if (SDL_MUSTLOCK(screen_p))
	{
		if (SDL_LockSurface(screen_p) < 0)
		{
			throw std::runtime_error("Failed to lock SDL screen");
		}
	}
	SDL_BlitSurface(surface_p, NULL, screen_p, NULL);
	if (SDL_MUSTLOCK(screen_p))
	{
		SDL_UnlockSurface(screen_p);
	}
	SDL_UpdateWindowSurface(window_p);
	SDL_FreeSurface(surface_p);
	return;
}

void convert_sensor_to_gil(const sensor_msgs::Image& src, boost::gil::rgb8_view_t& dst)
{
	boost::gil::rgb8_view_t temp = boost::gil::interleaved_view(src.width, src.height, (boost::gil::rgb8_pixel_t*)src.data.data(), src.width*3*sizeof(uint8_t));
	boost::gil::copy_and_convert_pixels(temp, dst);
	return;
}



void left_image_callback(const sensor_msgs::Image& left_image)
{
	ROS_INFO("Left Image encoding : %s", left_image.encoding.c_str());
	//The encoding is "bgr8"

	if(gui_initialized == false) init_gui(left_image.width, left_image.height);

	convert_sensor_to_gil(left_image, screen_left_view);

	update_gui();

	return;
}

void right_image_callback(const sensor_msgs::Image& right_image)
{
	ROS_INFO("Right Image encoding : %s", right_image.encoding.c_str());
	
	if( !gui_initialized ) init_gui(right_image.width, right_image.height);

	convert_sensor_to_gil(right_image, screen_right_view);

	update_gui();

	return;
}



int main(int argc, char** argv)
{
	//ROS setup
	ros::init(argc, argv, "print_image_node");
	ros::NodeHandle _nh("~");

	ros::Subscriber left_image_subscriber = _nh.subscribe("left_image",1,left_image_callback);
	ros::Subscriber right_image_subscriber = _nh.subscribe("right_image",1,right_image_callback);

	ros::ServiceClient next_frame_client = _nh.serviceClient<std_srvs::Empty>("next_frame_service");

	std_srvs::Empty empty_service;

	ROS_DEBUG("print_image_node Initialized");

	while(ros::ok())
	{
		ros::Duration(3.0).sleep(); //Create a duration object of 5 seconds, and sleep for that time
		next_frame_client.call(empty_service); //Tells it to load the next image
		ros::spinOnce();
	}

	return 0;
}