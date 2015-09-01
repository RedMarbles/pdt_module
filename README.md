pdt_module
===========

This project is a ROS implementation of the doppia pedestrian detection library built by Benenson et al.

When initializing this project, make sure you clone the doppia fork [pdt_360deg_git](https://github.com/RedMarbles/pdt_360deg_git) to the libs/ folder first.


Instructions for first-time use:
---------------------------------

The system requirements for this project are:

- Linux
- C++ compilation environments properly set. Only gcc 4.5 or superior are supported.
- CUDA compilation environment properly set. Make sure nvcc can be used to compile and run code.
- A GPU with CUDA capability 2.0 or higher.
- All boost libraries
- Google Protocol Buffer (all trained models are stored in this format)
- OpenCV installed
- libjpeg, libpng
- libSDL2
- CMake (verion 2.8.3 or greater) (You will need some experience working with CMake to get this to compile)

### Step 1 : Compile pdt_360_git

As mentioned earlier, this project is a ROS wrapper for the doppia library. Hence, the first step is to make sure the `pdt_360_git` library is present in the `pdt_module/libs` folder.

Make sure the `pdt_360_git` library compiles separately. This requires proper setup of that library. Instructions for compiling the code can be found in the `pdt_360_git/readme.text` file. A small summary of the requirements is:

- First, edit the file `pdt_360_git/common_settings.cmake`. It is mandatory to add a configuration that is specific to your system:
	- On the bash shell, run the command `hostname` to get the name of your system. Then replace this name in the `elseif` block at line 342 of the file. 
	- In the contents of the `elseif` block, add any configuration parameters specific to your system. For example, it is recommended to `local_CUDA_CUT_INCLUDE_DIRS` and `local_CUDA_CUT_LIBRARY_DIRS` to the version specific to your system.
	
- Execute (only once) the `generate_protocol_buffer_files.sh` script to generate the Google Protocol Buffer files according to the version installed on your system.

- Try to compile the applications present in `pdt_360_git/src/applications/` to make sure they compile without any problems. It is recommended to follow the instructions present in the `pdt_360_git/readme.text` file at steps 2 and 3. It is also recommended to use a graphical cmake tool, like `ccmake` instead of `cmake`, in order to check and tweak the compilation parameters.
	
### Step 2 : Compile pdt_module

If the `pdt_360_git` library applications compile and run perfectly, then the pdt_module applications are ready to be compiled and run. A few final tweaks that need to be made:

First, in the `pdt_module/CMakeFiles.txt` file:

- At line 62, the specific architecture of your NVIDIA GPU needs to be set. For my system, it was the `-arch compute_20 -code sm_20` instruction, because my CUDA compute capability is 2.0. For example, if the compute capability of your GPU is 3.5, you would set the flags `-arch compute_35 -code sm_35`.

- At line 77, the default build mode is `RelWithDebInfo`, which provides sub-optimal code that has some debug functionality. Change this to `Release` or `Debug` based on your requirements.

- At line 200, it is recommended to set the location of your local eigen2 libraries, if it is not the same as specified here.

- From line 288 onwards, the main nodes of pdt_module are compiled. Each node is encased within a block, and if you want the node to be compiled, set the respective `BUILD_...` flag to `ON`. The current nodes are:

	- ground_estimation_node : The same application as `pdt_module/libs/pdt_360_git/src/applications/ground_estimation`, but built to compile within ROS. This has only been left here for debug purposes, and is OFF by default.
	
	- stixel_world_node : The same application as `pdt_module/libs/pdt_360_git/src/applications/stixel_world`, but built to compile within ROS. This has only been left here for debug purposes, and is OFF by default.
	
	- objects_detection_node : The same application as `pdt_module/libs/pdt_360_git/src/applications/objects_detection`, but built to compile within ROS. This has only been left here for debug purposes, and is OFF by default.
	
	- image_read_node : A test application to read the output from the vehicle's camera topics and output the images. This has only been left here for debug purposes, and is OFF by default.
	
	- image_test_node : A test application to test some concepts and publishing ideas. This has only been left here for debug purposes, and is OFF by default.
	
	- **video_input_node** : One of the three main nodes of this package. This node accepts input from the stereo camera topics of the vehicle, preprocesses them, and publishes the rectified and resampled images.
	
	- **object_detector_node** : One of the three main nodes of this package. This node accepts the preprocessed stereo images and searches for pedestrians within them.
	
	- **stixels_estimator_node** : One of the three main nodes of this package. This node accepts the preprocessed stereo images and extracts the stixel information from the image, which is output to the stixels topic.
	
	
Additionally, there are some configuration files that need to be modified as well:

- `launch/video_input_node.launch` : Make sure the calibration files for the video match the camera calibrations.

- `launch/stixels_estimation_node.launch` : The stixels_estimator_node needs some parameters that need to be accepted in the form of a boost configuration file. The default file, found at `pdt_module/src/stixels_estimator/test_objects_detection_lib.config.ini`, though there really isn't anything in the file that needs to be changed.  
It also requires the camera calibration parameters in the boost format, and the sample file is available at `pdt_module/src/stixels_estimator/stereo_calibration_dataset1.proto.txt`.

- `launch/object_detector_node.launch` : This node also needs the boost configuration format, and the default file is  `pdt_module/src/object_detector/test_monocular_objects_detection_lib.config.ini`.  
**Important** : At line 19, make sure the name of the trained model on to use in this file has been appropriately renamed, as it was not possible to use a relative filename at this section.