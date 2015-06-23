
#include "BasicSdlGui.hpp"

#include <stdexcept>  // for std::cerr

namespace pdt_module
{

void BasicSdlGui::init_gui(const int input_width, const int input_height)
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

void BasicSdlGui::set_stereo_output(boost::gil::rgb8c_view_t left_image, boost::gil::rgb8c_view_t right_image)
{
	if(!gui_initialized)
	{
		init_gui(left_image.width(), left_image.height());
	}
	boost::gil::copy_and_convert_pixels(left_image, screen_left_view);
	boost::gil::copy_and_convert_pixels(right_image, screen_right_view);
}

void BasicSdlGui::update_gui()
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

} //exit pdt_module namespace