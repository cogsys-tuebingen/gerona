cmake_minimum_required(VERSION 2.4.6)

# Build library
add_library( LibLaserProcessing
    segmentation/distance_segmentation.cpp
    person_filter/leg_detector.cpp
    person_filter/person_detector.cpp
    person_filter/person_filter.cpp
)
