# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/CSE276A_Fall24/cse276a_ws/src/build/ros2_april_detection

# Include any dependencies generated for this target.
include CMakeFiles/april_detection_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/april_detection_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/april_detection_node.dir/flags.make

CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.o: CMakeFiles/april_detection_node.dir/flags.make
CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.o: /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection/src/april_detection_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/CSE276A_Fall24/cse276a_ws/src/build/ros2_april_detection/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.o -c /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection/src/april_detection_node.cpp

CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection/src/april_detection_node.cpp > CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.i

CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection/src/april_detection_node.cpp -o CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.s

CMakeFiles/april_detection_node.dir/src/april_detection.cpp.o: CMakeFiles/april_detection_node.dir/flags.make
CMakeFiles/april_detection_node.dir/src/april_detection.cpp.o: /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection/src/april_detection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/CSE276A_Fall24/cse276a_ws/src/build/ros2_april_detection/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/april_detection_node.dir/src/april_detection.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/april_detection_node.dir/src/april_detection.cpp.o -c /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection/src/april_detection.cpp

CMakeFiles/april_detection_node.dir/src/april_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/april_detection_node.dir/src/april_detection.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection/src/april_detection.cpp > CMakeFiles/april_detection_node.dir/src/april_detection.cpp.i

CMakeFiles/april_detection_node.dir/src/april_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/april_detection_node.dir/src/april_detection.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection/src/april_detection.cpp -o CMakeFiles/april_detection_node.dir/src/april_detection.cpp.s

# Object files for target april_detection_node
april_detection_node_OBJECTS = \
"CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.o" \
"CMakeFiles/april_detection_node.dir/src/april_detection.cpp.o"

# External object files for target april_detection_node
april_detection_node_EXTERNAL_OBJECTS =

april_detection_node: CMakeFiles/april_detection_node.dir/src/april_detection_node.cpp.o
april_detection_node: CMakeFiles/april_detection_node.dir/src/april_detection.cpp.o
april_detection_node: CMakeFiles/april_detection_node.dir/build.make
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/libstatic_transform_broadcaster_node.so
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
april_detection_node: /root/CSE276A_Fall24/cse276a_ws/src/install/cv_bridge/lib/libcv_bridge.so
april_detection_node: /opt/ros/foxy/lib/libapriltag.so.3.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_face.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_hfs.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_quality.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_tracking.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.2.0
april_detection_node: /opt/ros/foxy/lib/libtf2_ros.so
april_detection_node: /opt/ros/foxy/lib/libtf2.so
april_detection_node: /opt/ros/foxy/lib/libmessage_filters.so
april_detection_node: /opt/ros/foxy/lib/librclcpp_action.so
april_detection_node: /opt/ros/foxy/lib/librcl_action.so
april_detection_node: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libtf2_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libaction_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/libcomponent_manager.so
april_detection_node: /opt/ros/foxy/lib/librclcpp.so
april_detection_node: /opt/ros/foxy/lib/liblibstatistics_collector.so
april_detection_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/librcl.so
april_detection_node: /opt/ros/foxy/lib/librmw_implementation.so
april_detection_node: /opt/ros/foxy/lib/librmw.so
april_detection_node: /opt/ros/foxy/lib/librcl_logging_spdlog.so
april_detection_node: /usr/lib/aarch64-linux-gnu/libspdlog.so.1.5.0
april_detection_node: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
april_detection_node: /opt/ros/foxy/lib/libyaml.so
april_detection_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/libtracetools.so
april_detection_node: /opt/ros/foxy/lib/libament_index_cpp.so
april_detection_node: /opt/ros/foxy/lib/libclass_loader.so
april_detection_node: /opt/ros/foxy/lib/aarch64-linux-gnu/libconsole_bridge.so.1.0
april_detection_node: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
april_detection_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
april_detection_node: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
april_detection_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
april_detection_node: /opt/ros/foxy/lib/librosidl_typesupport_c.so
april_detection_node: /opt/ros/foxy/lib/librosidl_runtime_c.so
april_detection_node: /opt/ros/foxy/lib/librcpputils.so
april_detection_node: /opt/ros/foxy/lib/librcutils.so
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_text.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.2.0
april_detection_node: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.2.0
april_detection_node: CMakeFiles/april_detection_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/CSE276A_Fall24/cse276a_ws/src/build/ros2_april_detection/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable april_detection_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/april_detection_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/april_detection_node.dir/build: april_detection_node

.PHONY : CMakeFiles/april_detection_node.dir/build

CMakeFiles/april_detection_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/april_detection_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/april_detection_node.dir/clean

CMakeFiles/april_detection_node.dir/depend:
	cd /root/CSE276A_Fall24/cse276a_ws/src/build/ros2_april_detection && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection /root/CSE276A_Fall24/cse276a_ws/src/rb5_ros2/ros2_april_detection /root/CSE276A_Fall24/cse276a_ws/src/build/ros2_april_detection /root/CSE276A_Fall24/cse276a_ws/src/build/ros2_april_detection /root/CSE276A_Fall24/cse276a_ws/src/build/ros2_april_detection/CMakeFiles/april_detection_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/april_detection_node.dir/depend
