# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/ruixinhuang/p2-meshedit-hilary217

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug

# Include any dependencies generated for this target.
include CGL/deps/glew/CMakeFiles/glew.dir/depend.make

# Include the progress variables for this target.
include CGL/deps/glew/CMakeFiles/glew.dir/progress.make

# Include the compile flags for this target's objects.
include CGL/deps/glew/CMakeFiles/glew.dir/flags.make

CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o: CGL/deps/glew/CMakeFiles/glew.dir/flags.make
CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o: ../CGL/deps/glew/src/glew.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glew.dir/src/glew.c.o   -c /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew/src/glew.c

CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glew.dir/src/glew.c.i"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew/src/glew.c > CMakeFiles/glew.dir/src/glew.c.i

CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glew.dir/src/glew.c.s"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew/src/glew.c -o CMakeFiles/glew.dir/src/glew.c.s

CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o: CGL/deps/glew/CMakeFiles/glew.dir/flags.make
CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o: ../CGL/deps/glew/src/glewinfo.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glew.dir/src/glewinfo.c.o   -c /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew/src/glewinfo.c

CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glew.dir/src/glewinfo.c.i"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew/src/glewinfo.c > CMakeFiles/glew.dir/src/glewinfo.c.i

CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glew.dir/src/glewinfo.c.s"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew/src/glewinfo.c -o CMakeFiles/glew.dir/src/glewinfo.c.s

CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o: CGL/deps/glew/CMakeFiles/glew.dir/flags.make
CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o: ../CGL/deps/glew/src/visualinfo.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glew.dir/src/visualinfo.c.o   -c /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew/src/visualinfo.c

CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glew.dir/src/visualinfo.c.i"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew/src/visualinfo.c > CMakeFiles/glew.dir/src/visualinfo.c.i

CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glew.dir/src/visualinfo.c.s"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew/src/visualinfo.c -o CMakeFiles/glew.dir/src/visualinfo.c.s

# Object files for target glew
glew_OBJECTS = \
"CMakeFiles/glew.dir/src/glew.c.o" \
"CMakeFiles/glew.dir/src/glewinfo.c.o" \
"CMakeFiles/glew.dir/src/visualinfo.c.o"

# External object files for target glew
glew_EXTERNAL_OBJECTS =

CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o
CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o
CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o
CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/build.make
CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking C static library libglew.a"
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && $(CMAKE_COMMAND) -P CMakeFiles/glew.dir/cmake_clean_target.cmake
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glew.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CGL/deps/glew/CMakeFiles/glew.dir/build: CGL/deps/glew/libglew.a

.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/build

CGL/deps/glew/CMakeFiles/glew.dir/clean:
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew && $(CMAKE_COMMAND) -P CMakeFiles/glew.dir/cmake_clean.cmake
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/clean

CGL/deps/glew/CMakeFiles/glew.dir/depend:
	cd /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/ruixinhuang/p2-meshedit-hilary217 /Users/ruixinhuang/p2-meshedit-hilary217/CGL/deps/glew /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew /Users/ruixinhuang/p2-meshedit-hilary217/cmake-build-debug/CGL/deps/glew/CMakeFiles/glew.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/depend

