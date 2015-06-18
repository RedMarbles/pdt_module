file(GLOB stixels_estimator_SrcCpp
	"${pdt_module_root}/src/stixels_estimator/stixels_estimator_node.cpp"
)

set(stixels_estimator_LinkLibs
	${Boost_LIBRARIES}
	${SDL2_LIBRARIES}
	${libpng_LIBRARIES}
	${opencv_LIBRARIES}
)