cmake_minimum_required(VERSION 2.4.6)


rosbuild_add_library( LibLaserProcessing
    segmentation/distance_segmentation.cpp
    leg_filter/leg_detector.cpp
)
