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
CMAKE_SOURCE_DIR = /root/spacedrone/src/coords

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/spacedrone/build/coords

# Utility rule file for coords_uninstall.

# Include any custom commands dependencies for this target.
include CMakeFiles/coords_uninstall.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/coords_uninstall.dir/progress.make

CMakeFiles/coords_uninstall:
	/usr/bin/cmake -P /root/spacedrone/build/coords/ament_cmake_uninstall_target/ament_cmake_uninstall_target.cmake

coords_uninstall: CMakeFiles/coords_uninstall
coords_uninstall: CMakeFiles/coords_uninstall.dir/build.make
.PHONY : coords_uninstall

# Rule to build all files generated by this target.
CMakeFiles/coords_uninstall.dir/build: coords_uninstall
.PHONY : CMakeFiles/coords_uninstall.dir/build

CMakeFiles/coords_uninstall.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/coords_uninstall.dir/cmake_clean.cmake
.PHONY : CMakeFiles/coords_uninstall.dir/clean

CMakeFiles/coords_uninstall.dir/depend:
	cd /root/spacedrone/build/coords && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/spacedrone/src/coords /root/spacedrone/src/coords /root/spacedrone/build/coords /root/spacedrone/build/coords /root/spacedrone/build/coords/CMakeFiles/coords_uninstall.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/coords_uninstall.dir/depend

