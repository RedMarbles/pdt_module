add_definitions(-DOBJECT_DETECTOR_NODE)

file(GLOB object_detector_SrcCpp
	"${pdt_module_root}/src/object_detector/object_detector_node.cpp"
	"${pdt_module_root}/src/object_detector/ObjectDetectorBasic.cpp"
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
)

file(GLOB object_detector_MonocularSrcCpp

	#"${doppia_src}/objects_detection/*.c*"
	"${doppia_src}/objects_detection/Abstract*.c*"
	"${doppia_src}/objects_detection/*Converter.c*"
	"${doppia_src}/objects_detection/Base*.c*"
	"${doppia_src}/objects_detection/*Factory.c*"
	"${doppia_src}/objects_detection/Greedy*.c*"
	"${doppia_src}/objects_detection/Detection*.c*"
	"${doppia_src}/objects_detection/*Model.c*"
	"${doppia_src}/objects_detection/*Stage.c*"
	"${doppia_src}/objects_detection/Integral*.c*"
	"${doppia_src}/objects_detection/VeryFastIntegral*.c*"
	"${doppia_src}/objects_detection/SearchRange*.c*"
	"${doppia_src}/objects_detection/MultiscalesIntegral*.c*"
	"${doppia_src}/objects_detection/FastestPedestrian*.c*"
	"${doppia_src}/objects_detection/DetectorSearchRange.c*"
	"${doppia_src}/objects_detection/*.pb.c*"
	"${doppia_src}/objects_detection/cascade_stages/*.c*"
	"${doppia_src}/objects_detection/non_maximal_suppression/*.c*"
	"${doppia_src}/objects_detection/integral_channels/Integral*.cpp"
	"${doppia_src}/objects_detection/integral_channels/AbstractIntegral*.cpp"
	"${doppia_src}/objects_detection/integral_channels/AbstractChannels*.cpp"

	"${doppia_stereo}/ground_plane/Abstract*.cpp"
	"${doppia_stereo}/stixels/Abstract*.cpp"

	"${doppia_src}/video_input/Abstract*.cpp"
	"${doppia_src}/video_input/Metric*.cpp"
	"${doppia_src}/video_input/VideoFromCam.cpp"
	"${doppia_src}/video_input/calibration/*.c*"
	"${doppia_src}/video_input/preprocessing/AddBorderFunctor.cpp"
)

file(GLOB object_detector_StereoSrcCpp

	#"${doppia_stereo}/*.cpp"
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
)

file(GLOB object_detector_HelpersCpp
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

file(GLOB object_detector_SrcGpuCpp

  "${doppia_src}/objects_detection/Gpu*.cpp"
  "${doppia_src}/objects_detection/integral_channels/AbstractGpu*.cpp"
  "${doppia_src}/objects_detection/integral_channels/Gpu*.cpp"
  "${doppia_src}/helpers/gpu/*.cpp"

  #"${doppia_stereo}/SimpleTreesGpuStereo.cpp"
)

file(GLOB object_detector_SrcCuda

	"${doppia_src}/objects_detection/integral_channels/gpu/*.c*"
	"${doppia_src}/objects_detection/gpu/*.c*"
	# "${doppia_src}/helpers/gpu/*.cu"

	# "${doppia_stereo}/*.cu.c*"
	# "${doppia_stereo}/*.cu"
	# "${doppia_stereo}/gpu/*.cu.c*"
	# "${doppia_stereo}/gpu/*.cu"
)

set(object_detector_LinkLibs
	${Boost_LIBRARIES}
	${SDL2_LIBRARIES}
	${libpng_LIBRARIES}
	${opencv_LIBRARIES}
	pthread
	gomp
	protobuf
	jpeg
)


list(REMOVE_ITEM object_detector_SrcCpp ${object_detector_SrcCuda}) # just in case

if(USE_GPU)

	# add GPU related source code to the executable list
	list(APPEND object_detector_SrcCpp ${object_detector_SrcGpuCpp})

	# # add GPU related libraries
	# list(APPEND opencv_LIBRARIES opencv_imgproc opencv_gpu)

	# # Compile CUDA stuff

	# cuda_include_directories(${local_CUDA_CUT_INCLUDE_DIRS})
	# cuda_include_directories(${CUDA_INCLUDE_DIRS} ${CUDA_CUT_INCLUDE_DIR} ${local_CUDA_CUT_INCLUDE_DIR})
	# link_directories(${local_CUDA_CUT_LIBRARY_DIRS})

	cuda_add_library(cuda_stuff_library ${object_detector_SrcCuda})
	target_link_libraries(cuda_stuff_library
	   ${CUDA_LIBRARIES}
	   #cutil
	)

	set(object_detector_LinkLibs
		${object_detector_LinkLibs}
		cuda_stuff_library
	)
endif(USE_GPU)

