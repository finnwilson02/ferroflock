# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_SOURCE_DIR = /home/finn/ferroflock/helper_functions

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/finn/ferroflock/helper_functions/build

# Include any dependencies generated for this target.
include CMakeFiles/keyboard_control.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/keyboard_control.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/keyboard_control.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/keyboard_control.dir/flags.make

CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.o: CMakeFiles/keyboard_control.dir/flags.make
CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.o: /home/finn/ferroflock/helper_functions/src/keyboard_control.cpp
CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.o: CMakeFiles/keyboard_control.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/finn/ferroflock/helper_functions/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.o -MF CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.o.d -o CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.o -c /home/finn/ferroflock/helper_functions/src/keyboard_control.cpp

CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/finn/ferroflock/helper_functions/src/keyboard_control.cpp > CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.i

CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/finn/ferroflock/helper_functions/src/keyboard_control.cpp -o CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.s

# Object files for target keyboard_control
keyboard_control_OBJECTS = \
"CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.o"

# External object files for target keyboard_control
keyboard_control_EXTERNAL_OBJECTS =

libkeyboard_control.a: CMakeFiles/keyboard_control.dir/src/keyboard_control.cpp.o
libkeyboard_control.a: CMakeFiles/keyboard_control.dir/build.make
libkeyboard_control.a: CMakeFiles/keyboard_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/finn/ferroflock/helper_functions/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libkeyboard_control.a"
	$(CMAKE_COMMAND) -P CMakeFiles/keyboard_control.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/keyboard_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/keyboard_control.dir/build: libkeyboard_control.a
.PHONY : CMakeFiles/keyboard_control.dir/build

CMakeFiles/keyboard_control.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/keyboard_control.dir/cmake_clean.cmake
.PHONY : CMakeFiles/keyboard_control.dir/clean

CMakeFiles/keyboard_control.dir/depend:
	cd /home/finn/ferroflock/helper_functions/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/finn/ferroflock/helper_functions /home/finn/ferroflock/helper_functions /home/finn/ferroflock/helper_functions/build /home/finn/ferroflock/helper_functions/build /home/finn/ferroflock/helper_functions/build/CMakeFiles/keyboard_control.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/keyboard_control.dir/depend

