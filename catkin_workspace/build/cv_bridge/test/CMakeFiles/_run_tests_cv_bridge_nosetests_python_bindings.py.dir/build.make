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
CMAKE_SOURCE_DIR = /home/zhongliang/yuangao/roscode/catkin_workspace/src/vision_opencv/cv_bridge

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge

# Utility rule file for _run_tests_cv_bridge_nosetests_python_bindings.py.

# Include the progress variables for this target.
include test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/progress.make

test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py:
	cd /home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge/test && ../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/melodic/share/catkin/cmake/test/run_tests.py /home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge/test_results/cv_bridge/nosetests-python_bindings.py.xml "\"/usr/bin/cmake\" -E make_directory /home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge/test_results/cv_bridge" "/home/zhongliang/anaconda3/bin/nosetests -P --process-timeout=60 /home/zhongliang/yuangao/roscode/catkin_workspace/src/vision_opencv/cv_bridge/test/python_bindings.py --with-xunit --xunit-file=/home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge/test_results/cv_bridge/nosetests-python_bindings.py.xml"

_run_tests_cv_bridge_nosetests_python_bindings.py: test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py
_run_tests_cv_bridge_nosetests_python_bindings.py: test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/build.make

.PHONY : _run_tests_cv_bridge_nosetests_python_bindings.py

# Rule to build all files generated by this target.
test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/build: _run_tests_cv_bridge_nosetests_python_bindings.py

.PHONY : test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/build

test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/clean:
	cd /home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge/test && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/clean

test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/depend:
	cd /home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhongliang/yuangao/roscode/catkin_workspace/src/vision_opencv/cv_bridge /home/zhongliang/yuangao/roscode/catkin_workspace/src/vision_opencv/cv_bridge/test /home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge /home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge/test /home/zhongliang/yuangao/roscode/catkin_workspace/build/cv_bridge/test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/_run_tests_cv_bridge_nosetests_python_bindings.py.dir/depend

