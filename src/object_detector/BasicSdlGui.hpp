
#ifndef _BASICSDLGUI_HPP_
#define _BASICSDLGUI_HPP_


#include <SDL2/SDL.h>

#include <boost/gil/gil_all.hpp>

namespace pdt_module
{

class BasicSdlGui
{
private:
	SDL_Surface *screen_p;
	SDL_Window *window_p;
	boost::gil::rgb8_image_t screen_image;
	boost::gil::rgb8_view_t screen_image_view;
	boost::gil::rgb8_view_t screen_left_view, screen_right_view;

	bool gui_initialized;
public:
	void init_gui(const int input_width, const int input_height);

	void set_stereo_output(boost::gil::rgb8c_view_t left_image, boost::gil::rgb8c_view_t right_image);
	
	void update_gui();
};

} //exit pdt_module namespace

#endif 
//endblock _BASICSDLGUI_HPP_