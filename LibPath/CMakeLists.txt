#cmake_minimum_required(VERSION 2.4.6)
#if(COMMAND cmake_policy)
#  cmake_policy(SET CMP0003 NEW)
#endif(COMMAND cmake_policy)

#include_directories(
#    $ENV{RABOT}/Utils/LibUtil
#    $ENV{RABOT}/ros/eigen/include
#    )

#link_directories($ENV{RABOT}/lib)

rosbuild_add_library(LibPath  ReedsShepp/CurveGenerator.cpp
    ReedsShepp/Curve.cpp
    ReedsShepp/CurveSegment.cpp
    ReedsShepp/CircleSegment.cpp
    ReedsShepp/LineSegment.cpp
    ReedsShepp/GeometryHelper.cpp
    sampling/GoalRegion.cpp
    sampling/RingGoalRegion.cpp
    sampling/PoseListGoalRegion.cpp
    sampling/CentroidRadiusGoalRegion.cpp
    sampling/SamplingPlanner.cpp
    common/MapMath.cpp
    common/Point2d.cpp
    common/Pose2d.cpp
    common/Path.cpp
    common/GridMap2d.cpp
    common/SimpleGridMap2d.cpp
    common/Bresenham2d.cpp
    a_star/AStar.cpp
    a_star/Node.cpp
    a_star/OpenList.cpp
    generic/Common.hpp
    generic/SearchAlgorithm.hpp
    generic/Algorithms.hpp
    generic/Heuristics.hpp
    generic/Neighborhood.hpp
    generic/MapManager.hpp
    generic/NodeManager.hpp
    evaluation/PathRenderer.hpp
    evaluation/MapRenderer.hpp
    evaluation/Util.hpp
)

target_link_libraries(LibPath LibUtil)

#target_link_libraries(ReedsShepp    LibUtil)

#install(TARGETS ReedsShepp LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

find_package(OpenCV)
if(NOT OpenCV_FOUND)
    message( "OpenCV NOT found, not building test program" )

else(NOT OpenCV_FOUND)


    rosbuild_add_executable (CostPrecomputation
                             ReedsShepp/CostPrecomputation.cpp
                             ReedsShepp/CurveRenderer.cpp)
    target_link_libraries (CostPrecomputation
        LibUtil
        LibPath
        ${OpenCV_LIBS}
    )


    rosbuild_add_executable (ReedsSheppCurveTest
                             ReedsShepp/CurveTest.cpp
                             ReedsShepp/CurveRenderer.cpp)
    target_link_libraries (ReedsSheppCurveTest
        LibUtil
        LibPath
        ${OpenCV_LIBS}
    )
    rosbuild_add_executable (Path4wsSimulator
                             common/path4ws_simulator.cpp
                             common/path4ws_sim.cpp
    )
    target_link_libraries (Path4wsSimulator
        LibUtil
        LibPath
        ${OpenCV_LIBS}
    )
    rosbuild_add_executable (Evaluator
                             evaluation/evaluator.cpp)
    target_link_libraries (Evaluator
        LibUtil
        LibPath
        ${OpenCV_LIBS})



 #   install(TARGETS ReedsSheppCurveTest RUNTIME DESTINATION bin)
endif(NOT OpenCV_FOUND)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_BAK})

#add_executable(unit_tests ReedsShepp/UnitTest.cpp)

#target_link_libraries (unit_tests
#    LibUtil
#    ReedsShepp
#)
