
#ifndef _EMPTYGUI_HPP_
#define _EMPTYGUI_HPP_

#include "AbstractGui.hpp"

#include <boost/gil/gil_all.hpp>

namespace pdt_module
{

/**
 *  A dummy GUI class
 */
class EmptyGui : public AbstractGui
{
protected:
	/// No members

public:
	void init_gui(const int input_width, const int input_height)
	{
		/// Do nothing
	}

	void set_stereo_output(boost::gil::rgb8c_view_t left_image, boost::gil::rgb8c_view_t right_image)
	{
		/// Do nothing
	}
	
	void update_gui()
	{
		/// Do nothing
	}
};

} /// exit pdt_module namespace

#endif 
/// endblock _EMPTYGUI_HPP_