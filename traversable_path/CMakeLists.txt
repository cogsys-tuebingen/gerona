cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(Debug)

rosbuild_init()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(classify_terrain
    src/node_classify_terrain.cpp
    src/terrainclassifier.cpp
    src/calibrationdatastorage.cpp
    src/pointclassification.cpp
    src/point_types.h
    src/mapprocessor.cpp
    src/scancleaner.cpp
    src/scanfeaturecalculator.cpp
)
target_link_libraries(classify_terrain yaml-cpp)

rosbuild_add_executable(follow_path
    src/pathfollower.cpp
    src/point_types.h
    src/exceptions.h
    src/mapprocessor.cpp
    src/markerpublisher.cpp
    src/timeoutlocker.cpp
)

rosbuild_add_executable(display_path src/node_display_path.cpp src/visualization.cpp)

rosbuild_add_executable(get_steer_angles src/node_get_steer_angles.cpp)

rosbuild_add_executable(driven_distance src/node_driven_distance.cpp)

# create training data from logfiles
rosbuild_add_executable(training_data_extractor src/node_training_data_extractor.cpp src/training_data_extractor.cpp)

# for testing some simple things TODO: remove if not required any more
#rosbuild_add_executable(testblub src/test.cpp src/mapprocessor.cpp) # it seams, that an excutable with name 'test' makes problems...
