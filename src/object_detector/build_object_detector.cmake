file(GLOB object_detector_SrcCpp
	"${pdt_module_root}/src/object_detector/object_detector_node.cpp"
)

set(object_detector_LinkLibs
	${Boost_LIBRARIES}
	${SDL2_LIBRARIES}
	${libpng_LIBRARIES}
	${opencv_LIBRARIES}
)