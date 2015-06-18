file(GLOB image_read_SrcCpp
	"${pdt_module_root}/src/image_read/image_read.cpp"
)

set(image_read_LinkLibs
	${Boost_LIBRARIES}
	${SDL2_LIBRARIES}
	${libpng_LIBRARIES}
)