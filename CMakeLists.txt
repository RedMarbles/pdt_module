## Initialize cmake
cmake_minimum_required(VERSION 2.8.3)
project(pdt_module)

## Set the directory locations
set(pdt_module_root "${CMAKE_CURRENT_LIST_DIR}/")
set(doppia_root "${pdt_module_root}/libs/pdt_360deg_git/")
set(doppia_src "${doppia_root}/src")
set(doppia_stereo "${doppia_root}/src/stereo_matching")

## Set the cmake paths
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "./" ${doppia_root} ${CMAKE_MODULE_PATH})

## This package is required for the pkg_check_modules() cmake function
include(FindPkgConfig)

## Set this flag if you want to compile and use the CUDA code. Otherwise only the CPU versions of the code are compiled.
set(USE_GPU ON CACHE BOOL "Should the GPU be used ?" )

## This loads the local system settings from the doppia common_settings file
include(${doppia_root}/common_settings.cmake)


## Load the required modules
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  message_generation
)

pkg_check_modules(libpng REQUIRED libpng)
#pkg_check_modules(libjpeg REQUIRED libjpeg)
pkg_check_modules(opencv REQUIRED opencv>=2.3)

set(opencv_LIBRARIES
    opencv_core opencv_imgproc opencv_highgui opencv_ml
    opencv_video opencv_features2d
    opencv_calib3d
    #opencv_objdetect opencv_contrib
    opencv_legacy opencv_flann
    opencv_gpu
   ) # quick hack for opencv2.4 support

find_package(Boost REQUIRED  
   COMPONENTS program_options filesystem system thread
)

pkg_search_module(SDL2 REQUIRED sdl2)


## Set this flag if you want to compile and use the CUDA code. Otherwise only the CPU versions of the code are compiled.
# set(USE_GPU ON CACHE BOOL "Should the GPU be used ?" )
if(USE_GPU)
  find_package(CUDA 4.0 REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS} ${CUDA_CUT_INCLUDE_DIR} ${local_CUDA_CUT_INCLUDE_DIRS})
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch compute_20 -code sm_20)
  cuda_include_directories(
    "${doppia_root}/libs/" 
    ${CUDA_INCLUDE_DIRS} 
    ${CUDA_CUT_INCLUDE_DIR}
    ${local_CUDA_CUT_INCLUDE_DIR}
    ${local_CUDA_CUT_INCLUDE_DIRS}
    )
  link_directories(${local_CUDA_CUT_LIBRARY_DIRS})
endif(USE_GPU)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2") # enable sse2 even in debug mode

## Set this to the relevant build type. Choices are: { None | Debug | Release | RelWithDebInfo | MinSizeRel }
set(CMAKE_BUILD_TYPE "Debug")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FetchStereoImages.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pdt_module
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    cv_bridge
    image_transport
    message_filters
    message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
#	${catkin_INCLUDE_DIRS}
#)

# ----------------------------------------------------------------------
# Setup link and include directories

set(local_LIBRARY_DIRS
  "/usr/local/lib"
  "/usr/lib64"
  "/usr/lib64/atlas"
  "/usr/lib/sse2/atlas"
  "/usr/lib"     # Local library
  "/usr/lib32"   # Local 32-bit library
  ${local_CUDA_LIB_DIR}
)

set(local_INCLUDE_DIRS
  "${doppia_root}/libs"
  "${doppia_root}/src"
  "${doppia_root}/libs/cudatemplates/include"
  "${doppia_root}/src/applications/boosted_learning"
  "${doppia_root}/src/objects_detection/integral_channels" # to avoid doppia specific paths in the library header
  "${doppia_root}/src/objects_detection" # to avoid doppia specific paths in the library header
  "${doppia_stereo}/ground_plane"  # for protoc plane3d.pb.h
  "${pdt_module_root}/"
  "${pdt_module_root}/libs/"
  "/usr/include/eigen2/"
  "/usr/local/include/eigen2"
  "/usr/local/cuda/include"
)

link_directories(
  ${libpng_LIBRARY_DIRS}
  ${OpenEXR_LIBRARY_DIRS}
  ${opencv_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${local_LIBRARY_DIRS}
)

include_directories(
  # ${doppia_root}/libs
  # ${doppia_root}/src
  # ${doppia_stereo}/ground_plane  # for protoc plane3d.pb.h
  ${libpng_INCLUDE_DIRS}
  ${OpenEXR_INCLUDE_DIRS}
  ${opencv_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${local_INCLUDE_DIRS}
  # "${doppia_root}/libs/cudatemplates/include"
  ${SDL2_INCLUDE_DIRS}   # for SDL2 graphics libraries
  ${catkin_INCLUDE_DIRS} # for the ROS libraries
)


## Setup compile commands for each node

set(BUILD_GROUND_ESTIMATION OFF)# CACHE BOOL "Build the ground_estimation submodule?")
if(BUILD_GROUND_ESTIMATION)
	## Load the ground_estimation submodule
	include(${pdt_module_root}/src/ground_estimation/build_ground.cmake)
	add_executable(ground_estimation ${ground_estimation_SrcCpp})
	add_dependencies(ground_estimation pdt_module_gencpp)
	target_link_libraries(ground_estimation ${catkin_LIBRARIES} ${ground_estimation_LinkLibs})
endif(BUILD_GROUND_ESTIMATION)

set(BUILD_STIXEL_WORLD OFF)# CACHE BOOL "Build the stixel_world submodule?")
if(BUILD_STIXEL_WORLD)
  ## Load the stixel_world submodule
  include(${pdt_module_root}/src/stixel_world/build_stixel_world.cmake)
  add_executable(stixel_world ${stixel_world_SrcCpp})
  add_dependencies(stixel_world pdt_module_gencpp)
  target_link_libraries(stixel_world ${catkin_LIBRARIES} ${stixel_world_LinkLibs})
endif(BUILD_STIXEL_WORLD)

set(BUILD_OBJECTS_DETECTION OFF)# CACHE BOOL "Build the objects_detection submodule?")
if(BUILD_OBJECTS_DETECTION)
  ## Load the stixel_world submodule
  include(${pdt_module_root}/src/objects_detection/build_objects_detection.cmake)
  add_executable(objects_detection ${objects_detection_SrcMain})
  add_dependencies(objects_detection pdt_module_gencpp)
  target_link_libraries(objects_detection ${catkin_LIBRARIES} ${objects_detection_LinkLibs})
endif(BUILD_OBJECTS_DETECTION)

set(BUILD_IMAGE_READ OFF)# CACHE BOOL "Build the image_read submodule?")
if(BUILD_IMAGE_READ)
  ## Load the image_read module
  include(${pdt_module_root}/src/image_read/build_image_read.cmake)
  add_executable(image_read_node ${image_read_SrcCpp})
  add_dependencies(image_read_node pdt_module_gencpp)
  target_link_libraries(image_read_node ${catkin_LIBRARIES} ${image_read_LinkLibs})
endif(BUILD_IMAGE_READ)

set(BUILD_IMAGE_TEST OFF)# CACHE BOOL "Build the image_read submodule?")
if(BUILD_IMAGE_TEST)
  ## Load the image_read module
  include(${pdt_module_root}/src/image_test/build_image_test.cmake)
  add_executable(image_test_printer_node ${image_test_printer_SrcCpp})
  add_executable(image_test_reader_node ${image_test_reader_SrcCpp})
  add_dependencies(image_test_printer_node pdt_module_gencpp)
  add_dependencies(image_test_reader_node pdt_module_gencpp)
  target_link_libraries(image_test_printer_node ${catkin_LIBRARIES} ${image_test_LinkLibs})
  target_link_libraries(image_test_reader_node ${catkin_LIBRARIES} ${image_test_LinkLibs})
endif(BUILD_IMAGE_TEST)

set(BUILD_VIDEO_INPUT_NODE ON)# CACHE BOOL "Build the image_read submodule?")
if(BUILD_VIDEO_INPUT_NODE)
  ## Load the image_read module
  include(${pdt_module_root}/src/video_input/build_video_input_node.cmake)
  add_executable(video_input_node ${video_input_node_SrcCpp})
  add_dependencies(video_input_node pdt_module_gencpp)
  target_link_libraries(video_input_node ${catkin_LIBRARIES} ${video_input_node_LinkLibs})
endif(BUILD_VIDEO_INPUT_NODE)

set(BUILD_OBJECT_DETECTOR_NODE ON)
if(BUILD_OBJECT_DETECTOR_NODE)
  ##Load the object_detector module
  include(${pdt_module_root}/src/object_detector/build_object_detector.cmake)
  add_executable(object_detector_node 
    ${object_detector_SrcCpp}
    ${object_detector_MonocularSrcCpp}
    ${object_detector_StereoSrcCpp}
    ${object_detector_HelpersCpp}
  )
  add_dependencies(object_detector_node pdt_module_gencpp)
  target_link_libraries(object_detector_node ${catkin_LIBRARIES} ${object_detector_LinkLibs})
endif(BUILD_OBJECT_DETECTOR_NODE)

set(BUILD_STIXELS_ESTIMATOR_NODE OFF)
if(BUILD_STIXELS_ESTIMATOR_NODE)
  ##Load the stixels_estimator module
  include(${pdt_module_root}/src/stixels_estimator/build_stixels_estimator.cmake)
  add_executable(stixels_estimator_node ${stixels_estimator_SrcCpp})
  add_dependencies(stixels_estimator_node pdt_module_gencpp)
  target_link_libraries(stixels_estimator_node ${catkin_LIBRARIES} ${stixels_estimator_LinkLibs})
endif(BUILD_STIXELS_ESTIMATOR_NODE)

## Declare a cpp library
# add_library(pdt_module
#   src/${PROJECT_NAME}/pdt_module.cpp
# )

## Declare a cpp executable
# add_executable(pdt_module_node src/pdt_module_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pdt_module_node pdt_module_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(pdt_module_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pdt_module pdt_module_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pdt_module.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
