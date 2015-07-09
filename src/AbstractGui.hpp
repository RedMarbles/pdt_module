
#ifndef _ABSTRACTGUI_HPP_
#define _ABSTRACTGUI_HPP_

#include <boost/gil/gil_all.hpp>

namespace pdt_module
{

/**
 *  An abstract base class for all GUI classes to use
 */
class AbstractGui
{
protected:
	/// No members

public:
	virtual void init_gui(const int input_width, const int input_height) = 0;

	virtual void set_stereo_output(boost::gil::rgb8c_view_t left_image, boost::gil::rgb8c_view_t right_image) = 0;
	
	virtual void update_gui() = 0;
};

} /// exit pdt_module namespace

#endif 
/// endblock _ABSTRACTGUI_HPP_