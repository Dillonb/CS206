# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.4

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
CMAKE_SOURCE_DIR = /home/dillon/class/evorobies/bullet-2.82

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dillon/class/evorobies/bullet-2.82

# Include any dependencies generated for this target.
include Demos/RagdollDemo/CMakeFiles/simulator.dir/depend.make

# Include the progress variables for this target.
include Demos/RagdollDemo/CMakeFiles/simulator.dir/progress.make

# Include the compile flags for this target's objects.
include Demos/RagdollDemo/CMakeFiles/simulator.dir/flags.make

Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o: Demos/RagdollDemo/CMakeFiles/simulator.dir/flags.make
Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o: Demos/RagdollDemo/RagdollDemo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dillon/class/evorobies/bullet-2.82/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o"
	cd /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulator.dir/RagdollDemo.o -c /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo/RagdollDemo.cpp

Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulator.dir/RagdollDemo.i"
	cd /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo/RagdollDemo.cpp > CMakeFiles/simulator.dir/RagdollDemo.i

Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulator.dir/RagdollDemo.s"
	cd /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo/RagdollDemo.cpp -o CMakeFiles/simulator.dir/RagdollDemo.s

Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o.requires:

.PHONY : Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o.requires

Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o.provides: Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o.requires
	$(MAKE) -f Demos/RagdollDemo/CMakeFiles/simulator.dir/build.make Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o.provides.build
.PHONY : Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o.provides

Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o.provides.build: Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o


Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o: Demos/RagdollDemo/CMakeFiles/simulator.dir/flags.make
Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o: Demos/RagdollDemo/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dillon/class/evorobies/bullet-2.82/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o"
	cd /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulator.dir/main.o -c /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo/main.cpp

Demos/RagdollDemo/CMakeFiles/simulator.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulator.dir/main.i"
	cd /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo/main.cpp > CMakeFiles/simulator.dir/main.i

Demos/RagdollDemo/CMakeFiles/simulator.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulator.dir/main.s"
	cd /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo/main.cpp -o CMakeFiles/simulator.dir/main.s

Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o.requires:

.PHONY : Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o.requires

Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o.provides: Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o.requires
	$(MAKE) -f Demos/RagdollDemo/CMakeFiles/simulator.dir/build.make Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o.provides.build
.PHONY : Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o.provides

Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o.provides.build: Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o


# Object files for target simulator
simulator_OBJECTS = \
"CMakeFiles/simulator.dir/RagdollDemo.o" \
"CMakeFiles/simulator.dir/main.o"

# External object files for target simulator
simulator_EXTERNAL_OBJECTS =

Demos/RagdollDemo/simulator: Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o
Demos/RagdollDemo/simulator: Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o
Demos/RagdollDemo/simulator: Demos/RagdollDemo/CMakeFiles/simulator.dir/build.make
Demos/RagdollDemo/simulator: Demos/OpenGL/libOpenGLSupport.a
Demos/RagdollDemo/simulator: src/BulletDynamics/libBulletDynamics.a
Demos/RagdollDemo/simulator: src/BulletCollision/libBulletCollision.a
Demos/RagdollDemo/simulator: src/LinearMath/libLinearMath.a
Demos/RagdollDemo/simulator: /usr/lib64/libglut.so
Demos/RagdollDemo/simulator: /usr/lib64/libGL.so
Demos/RagdollDemo/simulator: /usr/lib64/libGLU.so
Demos/RagdollDemo/simulator: Demos/RagdollDemo/CMakeFiles/simulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dillon/class/evorobies/bullet-2.82/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable simulator"
	cd /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Demos/RagdollDemo/CMakeFiles/simulator.dir/build: Demos/RagdollDemo/simulator

.PHONY : Demos/RagdollDemo/CMakeFiles/simulator.dir/build

Demos/RagdollDemo/CMakeFiles/simulator.dir/requires: Demos/RagdollDemo/CMakeFiles/simulator.dir/RagdollDemo.o.requires
Demos/RagdollDemo/CMakeFiles/simulator.dir/requires: Demos/RagdollDemo/CMakeFiles/simulator.dir/main.o.requires

.PHONY : Demos/RagdollDemo/CMakeFiles/simulator.dir/requires

Demos/RagdollDemo/CMakeFiles/simulator.dir/clean:
	cd /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo && $(CMAKE_COMMAND) -P CMakeFiles/simulator.dir/cmake_clean.cmake
.PHONY : Demos/RagdollDemo/CMakeFiles/simulator.dir/clean

Demos/RagdollDemo/CMakeFiles/simulator.dir/depend:
	cd /home/dillon/class/evorobies/bullet-2.82 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dillon/class/evorobies/bullet-2.82 /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo /home/dillon/class/evorobies/bullet-2.82 /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo /home/dillon/class/evorobies/bullet-2.82/Demos/RagdollDemo/CMakeFiles/simulator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Demos/RagdollDemo/CMakeFiles/simulator.dir/depend
