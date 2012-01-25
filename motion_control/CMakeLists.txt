cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

# TODO: This is ugly. Find a better way to do this
#include_directories (
 #   $ENV{RABOT}/Utils/LibUtil
 #   $ENV{RABOT}/Utils/LibRobot
 #   $ENV{RABOT}/Utils/LibOdoCalib
 #   $ENV{RABOT}/Utils/LibBotHardware
 #   $ENV{RABOT}/ros/eigen/include
#
#)


link_directories(
    $ENV{RABOT}/lib
)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(follow_path
src/DualPidCtrl.cpp
src/PathFollowing.cpp
src/DualAxisDriver.cpp
)
target_link_libraries(follow_path LibUtil)


rosbuild_add_executable(drive_test
src/DriveTestNode.cpp
)

target_link_libraries(drive_test LibUtil)

rosbuild_add_executable(FollowTestNode
src/FollowTestNode.cpp
)

target_link_libraries(FollowTestNode LibUtil)


rosbuild_add_executable(samplingdrive_test
src/SamplingTestNode.cpp
)
target_link_libraries(drive_test LibUtil)

rosbuild_add_executable(motion_control
src/CalibDriver.cpp
src/DualPidCtrl.cpp
src/DualAxisDriver.cpp
src/FixedDriver.cpp
src/MotionControlNode.cpp
src/MotionController.cpp
src/PathDriver.cpp
src/RsPathDriver.cpp
src/SimpleGoalDriver.cpp

)

target_link_libraries(motion_control
LibOdoCalib
LibBotHardware
LibRobot
LibUtil
Gnuplot gsl gslcblas
)
