# Uses same Stixel fields as found in libs/pdt_360_git/src/stereo_matching/stixels/Stixel.hpp

# Width of the stixel
int32 width

# X-position of the center of the stixel
int32 x

# Y-position of the bottom of the stixel
int32 bottom_y

# Y-position of the top of the stixel
int32 top_y

# Was the height estimated using a default value or actual data?
bool default_height_value

# Depth corresponding to the nearest point covered by the stixel, in pixels
int32 disparity

# Originally an enum Types {Unknown, Occluded, Car, Pedestrian, StaticObject} type
int8 type

# These are about the horizontal displacement starting from the center of the 
# stixel and ending at the center of the matched stixel "backwards"
int32 backward_delta_x
bool valid_backward_delta_x

int32 backward_width
