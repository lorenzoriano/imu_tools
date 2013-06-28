cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# create library
rosbuild_add_library (imu_filter_stabilizer src/imu_filter_stabilizer.cpp)

# create nodelet library
rosbuild_add_library (imu_filter_stabilizer_nodelet src/imu_filter_stabilizer_nodelet.cpp)
target_link_libraries (imu_filter_stabilizer_nodelet imu_filter_stabilizer)

# create standalone executable
rosbuild_add_executable(imu_filter_stabilizer_node src/imu_filter_stabilizer_node.cpp)
target_link_libraries (imu_filter_stabilizer_node imu_filter_stabilizer)
