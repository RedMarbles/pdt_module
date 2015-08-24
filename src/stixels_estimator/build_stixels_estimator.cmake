add_definitions(-DSTIXELS_ESTIMATOR_NODE)

file(GLOB stixels_estimator_SrcCpp
	"${pdt_module_root}/src/stixels_estimator/stixels_estimator_node.cpp"
	"${pdt_module_root}/src/stixels_estimator/StixelsEstimatorBasic.cpp"
	"${pdt_module_root}/src/BasicSdlGui.cpp"

	"${doppia_src}/image_processing/*.cpp"
	"${doppia_src}/drawing/gil/draw_matrix.cpp"
	"${doppia_src}/drawing/gil/draw_the_detections.cpp"
	"${doppia_src}/drawing/gil/colors.cpp"
	"${doppia_src}/applications/objects_detection/draw*.cpp"
	"${doppia_src}/applications/stixel_world/draw*.cpp"
	"${doppia_src}/drawing/gil/*.cpp"

	"${doppia_src}/visual_odometry/Abstract*.cpp"
	"${doppia_src}/objects_tracking/*.cpp"
	"${doppia_src}/objects_tracking/motion_models/*.cpp"
	"${doppia_src}/objects_tracking/detection_descriptors/*.cpp"
	"${doppia_src}/objects_tracking/tracked_detections/*.cpp"

	"${doppia_src}/video_input/Abstract*.cpp"
	"${doppia_src}/video_input/Metric*.cpp"
	"${doppia_src}/video_input/VideoFromCam.cpp"
	"${doppia_src}/video_input/calibration/*.c*"
	"${doppia_src}/video_input/preprocessing/AddBorderFunctor.cpp"

	"${doppia_stereo}/cost_volume/*CostVolume.cpp"
	"${doppia_stereo}/cost_volume/*CostVolumeEstimator*.cpp"
	"${doppia_stereo}/cost_volume/DisparityCostVolumeFromDepthMap.cpp"
	"${doppia_stereo}/cost_functions.cpp"
	"${doppia_stereo}/CensusCostFunction.cpp"
	"${doppia_stereo}/CensusTransform.cpp"
	"${doppia_stereo}/GradientTransform.cpp"
	"${doppia_stereo}/AbstractStereoMatcher.cpp"
	"${doppia_stereo}/AbstractStereoBlockMatcher.cpp"
	"${doppia_stereo}/SimpleBlockMatcher.cpp"
	"${doppia_stereo}/MutualInformationCostFunction.cpp"
	"${doppia_stereo}/ConstantSpaceBeliefPropagation.cpp"
	"${doppia_stereo}/qingxiong_yang/*.cpp"
	"${doppia_stereo}/SimpleTreesOptimizationStereo.cpp"
	"${doppia_stereo}/OpenCvStereo.cpp"

	"${doppia_stereo}/ground_plane/*.cpp"
	"${doppia_stereo}/stixels/*.cpp"
	#"${doppia_stereo}/stixels/draw_stixel_world.cpp"
	#"${doppia_stereo}/objects_detection/draw_the_detections.cpp"
	#"${doppia_stereo}/stixels/*.cc"
	"${doppia_src}/video_input/*.cpp"
	"${doppia_src}/video_input/calibration/*.c*"
	"${doppia_src}/video_input/preprocessing/*Preprocessor.cpp"
	"${doppia_src}/video_input/preprocessing/*Mapper.cpp"

	"${doppia_src}/objects_detection/Detection*.c*"
)

file(GLOB stixels_estimator_HelpersCpp
  # "FakeObjectsDetectionLibGui.cpp"
  #"${doppia_src}/helpers/*.cpp"
  "${doppia_src}/helpers/data/*.c*"
  "${doppia_src}/helpers/any_to_string.cpp"
  "${doppia_src}/helpers/get_section_options.cpp"
  "${doppia_src}/helpers/Log.cpp"
  "${doppia_src}/helpers/ModuleLog.cpp"
  "${doppia_src}/helpers/loggers.cpp"
  "${doppia_src}/helpers/AlignedImage.cpp"
  "${doppia_src}/helpers/replace_environment_variables.cpp"
  "${doppia_src}/helpers/objects_detection/*.cpp"
  "${doppia_src}/helpers/Timers.cpp"
)

set(stixels_estimator_LinkLibs
	${Boost_LIBRARIES}
	${SDL2_LIBRARIES}
	${libpng_LIBRARIES}
	${opencv_LIBRARIES}
	pthread
	gomp
	protobuf
	jpeg
)