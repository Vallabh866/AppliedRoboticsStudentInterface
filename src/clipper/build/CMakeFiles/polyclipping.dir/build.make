# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/hendrik/workspace/project/src/clipper

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hendrik/workspace/project/src/clipper/build

# Include any dependencies generated for this target.
include CMakeFiles/polyclipping.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/polyclipping.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/polyclipping.dir/flags.make

CMakeFiles/polyclipping.dir/clipper.cpp.o: CMakeFiles/polyclipping.dir/flags.make
CMakeFiles/polyclipping.dir/clipper.cpp.o: ../clipper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hendrik/workspace/project/src/clipper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/polyclipping.dir/clipper.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/polyclipping.dir/clipper.cpp.o -c /home/hendrik/workspace/project/src/clipper/clipper.cpp

CMakeFiles/polyclipping.dir/clipper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/polyclipping.dir/clipper.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hendrik/workspace/project/src/clipper/clipper.cpp > CMakeFiles/polyclipping.dir/clipper.cpp.i

CMakeFiles/polyclipping.dir/clipper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/polyclipping.dir/clipper.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hendrik/workspace/project/src/clipper/clipper.cpp -o CMakeFiles/polyclipping.dir/clipper.cpp.s

CMakeFiles/polyclipping.dir/clipper.cpp.o.requires:

.PHONY : CMakeFiles/polyclipping.dir/clipper.cpp.o.requires

CMakeFiles/polyclipping.dir/clipper.cpp.o.provides: CMakeFiles/polyclipping.dir/clipper.cpp.o.requires
	$(MAKE) -f CMakeFiles/polyclipping.dir/build.make CMakeFiles/polyclipping.dir/clipper.cpp.o.provides.build
.PHONY : CMakeFiles/polyclipping.dir/clipper.cpp.o.provides

CMakeFiles/polyclipping.dir/clipper.cpp.o.provides.build: CMakeFiles/polyclipping.dir/clipper.cpp.o


# Object files for target polyclipping
polyclipping_OBJECTS = \
"CMakeFiles/polyclipping.dir/clipper.cpp.o"

# External object files for target polyclipping
polyclipping_EXTERNAL_OBJECTS =

libpolyclipping.so.22.0.0: CMakeFiles/polyclipping.dir/clipper.cpp.o
libpolyclipping.so.22.0.0: CMakeFiles/polyclipping.dir/build.make
libpolyclipping.so.22.0.0: CMakeFiles/polyclipping.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hendrik/workspace/project/src/clipper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libpolyclipping.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/polyclipping.dir/link.txt --verbose=$(VERBOSE)
	$(CMAKE_COMMAND) -E cmake_symlink_library libpolyclipping.so.22.0.0 libpolyclipping.so.22 libpolyclipping.so

libpolyclipping.so.22: libpolyclipping.so.22.0.0
	@$(CMAKE_COMMAND) -E touch_nocreate libpolyclipping.so.22

libpolyclipping.so: libpolyclipping.so.22.0.0
	@$(CMAKE_COMMAND) -E touch_nocreate libpolyclipping.so

# Rule to build all files generated by this target.
CMakeFiles/polyclipping.dir/build: libpolyclipping.so

.PHONY : CMakeFiles/polyclipping.dir/build

CMakeFiles/polyclipping.dir/requires: CMakeFiles/polyclipping.dir/clipper.cpp.o.requires

.PHONY : CMakeFiles/polyclipping.dir/requires

CMakeFiles/polyclipping.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/polyclipping.dir/cmake_clean.cmake
.PHONY : CMakeFiles/polyclipping.dir/clean

CMakeFiles/polyclipping.dir/depend:
	cd /home/hendrik/workspace/project/src/clipper/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hendrik/workspace/project/src/clipper /home/hendrik/workspace/project/src/clipper /home/hendrik/workspace/project/src/clipper/build /home/hendrik/workspace/project/src/clipper/build /home/hendrik/workspace/project/src/clipper/build/CMakeFiles/polyclipping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/polyclipping.dir/depend

