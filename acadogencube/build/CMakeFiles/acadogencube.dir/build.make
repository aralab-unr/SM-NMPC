# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vanchung/acadogen/acadogencube

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vanchung/acadogen/acadogencube/build

# Include any dependencies generated for this target.
include CMakeFiles/acadogencube.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/acadogencube.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/acadogencube.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/acadogencube.dir/flags.make

CMakeFiles/acadogencube.dir/src/acadogencube.cpp.o: CMakeFiles/acadogencube.dir/flags.make
CMakeFiles/acadogencube.dir/src/acadogencube.cpp.o: ../src/acadogencube.cpp
CMakeFiles/acadogencube.dir/src/acadogencube.cpp.o: CMakeFiles/acadogencube.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vanchung/acadogen/acadogencube/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/acadogencube.dir/src/acadogencube.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/acadogencube.dir/src/acadogencube.cpp.o -MF CMakeFiles/acadogencube.dir/src/acadogencube.cpp.o.d -o CMakeFiles/acadogencube.dir/src/acadogencube.cpp.o -c /home/vanchung/acadogen/acadogencube/src/acadogencube.cpp

CMakeFiles/acadogencube.dir/src/acadogencube.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/acadogencube.dir/src/acadogencube.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vanchung/acadogen/acadogencube/src/acadogencube.cpp > CMakeFiles/acadogencube.dir/src/acadogencube.cpp.i

CMakeFiles/acadogencube.dir/src/acadogencube.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/acadogencube.dir/src/acadogencube.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vanchung/acadogen/acadogencube/src/acadogencube.cpp -o CMakeFiles/acadogencube.dir/src/acadogencube.cpp.s

# Object files for target acadogencube
acadogencube_OBJECTS = \
"CMakeFiles/acadogencube.dir/src/acadogencube.cpp.o"

# External object files for target acadogencube
acadogencube_EXTERNAL_OBJECTS =

acadogencube: CMakeFiles/acadogencube.dir/src/acadogencube.cpp.o
acadogencube: CMakeFiles/acadogencube.dir/build.make
acadogencube: /usr/local/lib/libacado_toolkit_s.so
acadogencube: CMakeFiles/acadogencube.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vanchung/acadogen/acadogencube/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable acadogencube"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/acadogencube.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/acadogencube.dir/build: acadogencube
.PHONY : CMakeFiles/acadogencube.dir/build

CMakeFiles/acadogencube.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/acadogencube.dir/cmake_clean.cmake
.PHONY : CMakeFiles/acadogencube.dir/clean

CMakeFiles/acadogencube.dir/depend:
	cd /home/vanchung/acadogen/acadogencube/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vanchung/acadogen/acadogencube /home/vanchung/acadogen/acadogencube /home/vanchung/acadogen/acadogencube/build /home/vanchung/acadogen/acadogencube/build /home/vanchung/acadogen/acadogencube/build/CMakeFiles/acadogencube.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/acadogencube.dir/depend

