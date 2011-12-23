#cmake_minimum_required(VERSION 2.4.6)
#if(COMMAND cmake_policy)
#  cmake_policy(SET CMP0003 NEW)
#endif(COMMAND cmake_policy)

#include_directories(
#    $ENV{RABOT}/Utils/LibUtil
#    $ENV{RABOT}/ros/eigen/include
#    )

#link_directories($ENV{RABOT}/lib)

rosbuild_add_library(ReedsShepp  ReedsShepp/CurveGenerator.cpp
                        ReedsShepp/Curve.cpp
                        ReedsShepp/CurveSegment.cpp
                        ReedsShepp/CircleSegment.cpp
                        ReedsShepp/LineSegment.cpp
                        common/Map.cpp
                        common/MapMath.cpp
                        common/Point2d.cpp
                        common/Pose2d.cpp)

#target_link_libraries(ReedsShepp    LibUtil)

#install(TARGETS ReedsShepp LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

#if(NOT OpenCV_FOUND)
#    message( "OpenCV NOT found, not building test program" )

#else(NOT OpenCV_FOUND)
#    rosbuild_add_executable (ReedsSheppCurveTest ReedsShepp/CurveTest.cpp
#                                        ReedsShepp/CurveRenderer.cpp)

#    target_link_libraries (ReedsSheppCurveTest
#        LibUtil
#        ReedsShepp
#        ${OpenCV_LIBS}
#    )

 #   install(TARGETS ReedsSheppCurveTest RUNTIME DESTINATION bin)
#endif(NOT OpenCV_FOUND)


#add_executable(unit_tests ReedsShepp/UnitTest.cpp)

#target_link_libraries (unit_tests
#    LibUtil
#    ReedsShepp
#)
