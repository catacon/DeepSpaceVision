# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/aaron/FRC2019/DeepSpaceVision

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aaron/FRC2019/DeepSpaceVision/build

# Include any dependencies generated for this target.
include CMakeFiles/DeepSpaceVision.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/DeepSpaceVision.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DeepSpaceVision.dir/flags.make

CMakeFiles/DeepSpaceVision.dir/main.cpp.o: CMakeFiles/DeepSpaceVision.dir/flags.make
CMakeFiles/DeepSpaceVision.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aaron/FRC2019/DeepSpaceVision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/DeepSpaceVision.dir/main.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DeepSpaceVision.dir/main.cpp.o -c /home/aaron/FRC2019/DeepSpaceVision/main.cpp

CMakeFiles/DeepSpaceVision.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DeepSpaceVision.dir/main.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aaron/FRC2019/DeepSpaceVision/main.cpp > CMakeFiles/DeepSpaceVision.dir/main.cpp.i

CMakeFiles/DeepSpaceVision.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DeepSpaceVision.dir/main.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aaron/FRC2019/DeepSpaceVision/main.cpp -o CMakeFiles/DeepSpaceVision.dir/main.cpp.s

CMakeFiles/DeepSpaceVision.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/DeepSpaceVision.dir/main.cpp.o.requires

CMakeFiles/DeepSpaceVision.dir/main.cpp.o.provides: CMakeFiles/DeepSpaceVision.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/DeepSpaceVision.dir/build.make CMakeFiles/DeepSpaceVision.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/DeepSpaceVision.dir/main.cpp.o.provides

CMakeFiles/DeepSpaceVision.dir/main.cpp.o.provides.build: CMakeFiles/DeepSpaceVision.dir/main.cpp.o


CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o: CMakeFiles/DeepSpaceVision.dir/flags.make
CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o: ../DeepSpaceVision.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aaron/FRC2019/DeepSpaceVision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o -c /home/aaron/FRC2019/DeepSpaceVision/DeepSpaceVision.cpp

CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aaron/FRC2019/DeepSpaceVision/DeepSpaceVision.cpp > CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.i

CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aaron/FRC2019/DeepSpaceVision/DeepSpaceVision.cpp -o CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.s

CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o.requires:

.PHONY : CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o.requires

CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o.provides: CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o.requires
	$(MAKE) -f CMakeFiles/DeepSpaceVision.dir/build.make CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o.provides.build
.PHONY : CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o.provides

CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o.provides.build: CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o


CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o: CMakeFiles/DeepSpaceVision.dir/flags.make
CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o: ../TargetFinder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aaron/FRC2019/DeepSpaceVision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o -c /home/aaron/FRC2019/DeepSpaceVision/TargetFinder.cpp

CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aaron/FRC2019/DeepSpaceVision/TargetFinder.cpp > CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.i

CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aaron/FRC2019/DeepSpaceVision/TargetFinder.cpp -o CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.s

CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o.requires:

.PHONY : CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o.requires

CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o.provides: CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o.requires
	$(MAKE) -f CMakeFiles/DeepSpaceVision.dir/build.make CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o.provides.build
.PHONY : CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o.provides

CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o.provides.build: CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o


CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o: CMakeFiles/DeepSpaceVision.dir/flags.make
CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o: ../DataSender.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aaron/FRC2019/DeepSpaceVision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o -c /home/aaron/FRC2019/DeepSpaceVision/DataSender.cpp

CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aaron/FRC2019/DeepSpaceVision/DataSender.cpp > CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.i

CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aaron/FRC2019/DeepSpaceVision/DataSender.cpp -o CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.s

CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o.requires:

.PHONY : CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o.requires

CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o.provides: CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o.requires
	$(MAKE) -f CMakeFiles/DeepSpaceVision.dir/build.make CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o.provides.build
.PHONY : CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o.provides

CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o.provides.build: CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o


# Object files for target DeepSpaceVision
DeepSpaceVision_OBJECTS = \
"CMakeFiles/DeepSpaceVision.dir/main.cpp.o" \
"CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o" \
"CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o" \
"CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o"

# External object files for target DeepSpaceVision
DeepSpaceVision_EXTERNAL_OBJECTS =

DeepSpaceVision: CMakeFiles/DeepSpaceVision.dir/main.cpp.o
DeepSpaceVision: CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o
DeepSpaceVision: CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o
DeepSpaceVision: CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o
DeepSpaceVision: CMakeFiles/DeepSpaceVision.dir/build.make
DeepSpaceVision: /usr/local/lib/libopencv_dnn.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_gapi.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_ml.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_objdetect.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_photo.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_stitching.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_video.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_calib3d.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_features2d.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_flann.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_highgui.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_videoio.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_imgcodecs.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_imgproc.so.4.0.1
DeepSpaceVision: /usr/local/lib/libopencv_core.so.4.0.1
DeepSpaceVision: CMakeFiles/DeepSpaceVision.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aaron/FRC2019/DeepSpaceVision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable DeepSpaceVision"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DeepSpaceVision.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DeepSpaceVision.dir/build: DeepSpaceVision

.PHONY : CMakeFiles/DeepSpaceVision.dir/build

CMakeFiles/DeepSpaceVision.dir/requires: CMakeFiles/DeepSpaceVision.dir/main.cpp.o.requires
CMakeFiles/DeepSpaceVision.dir/requires: CMakeFiles/DeepSpaceVision.dir/DeepSpaceVision.cpp.o.requires
CMakeFiles/DeepSpaceVision.dir/requires: CMakeFiles/DeepSpaceVision.dir/TargetFinder.cpp.o.requires
CMakeFiles/DeepSpaceVision.dir/requires: CMakeFiles/DeepSpaceVision.dir/DataSender.cpp.o.requires

.PHONY : CMakeFiles/DeepSpaceVision.dir/requires

CMakeFiles/DeepSpaceVision.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/DeepSpaceVision.dir/cmake_clean.cmake
.PHONY : CMakeFiles/DeepSpaceVision.dir/clean

CMakeFiles/DeepSpaceVision.dir/depend:
	cd /home/aaron/FRC2019/DeepSpaceVision/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aaron/FRC2019/DeepSpaceVision /home/aaron/FRC2019/DeepSpaceVision /home/aaron/FRC2019/DeepSpaceVision/build /home/aaron/FRC2019/DeepSpaceVision/build /home/aaron/FRC2019/DeepSpaceVision/build/CMakeFiles/DeepSpaceVision.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/DeepSpaceVision.dir/depend

