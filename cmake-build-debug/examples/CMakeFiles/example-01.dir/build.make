# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/clion/178/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/178/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aldo/CLionProjects/maestro-haptics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug

# Include any dependencies generated for this target.
include examples/CMakeFiles/example-01.dir/depend.make
# Include the progress variables for this target.
include examples/CMakeFiles/example-01.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example-01.dir/flags.make

examples/CMakeFiles/example-01.dir/example-01/main.cpp.o: examples/CMakeFiles/example-01.dir/flags.make
examples/CMakeFiles/example-01.dir/example-01/main.cpp.o: ../examples/example-01/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/example-01.dir/example-01/main.cpp.o"
	cd /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example-01.dir/example-01/main.cpp.o -c /home/aldo/CLionProjects/maestro-haptics/examples/example-01/main.cpp

examples/CMakeFiles/example-01.dir/example-01/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-01.dir/example-01/main.cpp.i"
	cd /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldo/CLionProjects/maestro-haptics/examples/example-01/main.cpp > CMakeFiles/example-01.dir/example-01/main.cpp.i

examples/CMakeFiles/example-01.dir/example-01/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-01.dir/example-01/main.cpp.s"
	cd /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldo/CLionProjects/maestro-haptics/examples/example-01/main.cpp -o CMakeFiles/example-01.dir/example-01/main.cpp.s

examples/CMakeFiles/example-01.dir/example-01/my_app.cpp.o: examples/CMakeFiles/example-01.dir/flags.make
examples/CMakeFiles/example-01.dir/example-01/my_app.cpp.o: ../examples/example-01/my_app.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object examples/CMakeFiles/example-01.dir/example-01/my_app.cpp.o"
	cd /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example-01.dir/example-01/my_app.cpp.o -c /home/aldo/CLionProjects/maestro-haptics/examples/example-01/my_app.cpp

examples/CMakeFiles/example-01.dir/example-01/my_app.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-01.dir/example-01/my_app.cpp.i"
	cd /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldo/CLionProjects/maestro-haptics/examples/example-01/my_app.cpp > CMakeFiles/example-01.dir/example-01/my_app.cpp.i

examples/CMakeFiles/example-01.dir/example-01/my_app.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-01.dir/example-01/my_app.cpp.s"
	cd /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldo/CLionProjects/maestro-haptics/examples/example-01/my_app.cpp -o CMakeFiles/example-01.dir/example-01/my_app.cpp.s

# Object files for target example-01
example__01_OBJECTS = \
"CMakeFiles/example-01.dir/example-01/main.cpp.o" \
"CMakeFiles/example-01.dir/example-01/my_app.cpp.o"

# External object files for target example-01
example__01_EXTERNAL_OBJECTS =

examples/example-01: examples/CMakeFiles/example-01.dir/example-01/main.cpp.o
examples/example-01: examples/CMakeFiles/example-01.dir/example-01/my_app.cpp.o
examples/example-01: examples/CMakeFiles/example-01.dir/build.make
examples/example-01: libmaestro-haptics.a
examples/example-01: /usr/lib/x86_64-linux-gnu/libglfw.so.3.3
examples/example-01: /usr/lib/x86_64-linux-gnu/libGLX.so
examples/example-01: /usr/lib/x86_64-linux-gnu/libOpenGL.so
examples/example-01: glad/libglad.a
examples/example-01: libmaestro3.a
examples/example-01: libesmacat.a
examples/example-01: libsoem.a
examples/example-01: examples/CMakeFiles/example-01.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable example-01"
	cd /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example-01.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example-01.dir/build: examples/example-01
.PHONY : examples/CMakeFiles/example-01.dir/build

examples/CMakeFiles/example-01.dir/clean:
	cd /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples && $(CMAKE_COMMAND) -P CMakeFiles/example-01.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example-01.dir/clean

examples/CMakeFiles/example-01.dir/depend:
	cd /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldo/CLionProjects/maestro-haptics /home/aldo/CLionProjects/maestro-haptics/examples /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples /home/aldo/CLionProjects/maestro-haptics/cmake-build-debug/examples/CMakeFiles/example-01.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example-01.dir/depend

