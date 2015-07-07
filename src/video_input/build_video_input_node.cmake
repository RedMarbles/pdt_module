file(GLOB video_input_node_SrcCpp
	"${pdt_module_root}/src/video_input/video_input_node.cpp"
	#"${pdt_module_root}/src/video_input/VideoInputPrimitive.cpp"
	"${pdt_module_root}/src/video_input/VideoInputSynchronized.cpp"
	# "${pdt_module_root}/src/video_input/VideoInputVeryPrimitive.cpp"
	# "${pdt_module_root}/src/video_input/video2.cpp"
	"${pdt_module_root}/src/video_input/preprocessor/AbstractPreprocessor.cpp"
	"${pdt_module_root}/src/video_input/preprocessor/MeiModelPreprocessor.cpp"
	"${pdt_module_root}/src/video_input/preprocessor/ResamplePreprocessor.cpp"
	"${pdt_module_root}/src/video_input/preprocessor/EmptyPreprocessor.cpp"
	"${pdt_module_root}/src/video_input/preprocessor/camera_models/utils.cpp"
	"${pdt_module_root}/src/video_input/preprocessor/camera_models/utils_mei.cpp"
)

set(video_input_node_LinkLibs
	${Boost_LIBRARIES}
	${SDL2_LIBRARIES}
	${libpng_LIBRARIES}
	${opencv_LIBRARIES}
)