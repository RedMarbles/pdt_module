file(GLOB image_test_printer_SrcCpp
	"${pdt_module_root}/src/image_test/print_image.cpp"
)

file(GLOB image_test_reader_SrcCpp
	"${pdt_module_root}/src/image_test/read_from_camera.cpp"
)

set(image_test_LinkLibs
	${Boost_LIBRARIES}
	${SDL2_LIBRARIES}
	${libpng_LIBRARIES}
)