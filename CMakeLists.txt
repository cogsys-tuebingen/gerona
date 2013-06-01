#cmake_minimum_required(VERSION 2.4.6)
#if(COMMAND cmake_policy)
#  cmake_policy(SET CMP0003 NEW)
#endif(COMMAND cmake_policy)

#include_directories(
#    $ENV{RABOT}/Utils/LibUtil
#    $ENV{RABOT}/ros/eigen/include
#    )

#link_directories($ENV{RABOT}/lib)

find_package(Boost COMPONENTS program_options thread REQUIRED)

set(files ReedsShepp/CurveGenerator.cpp
    ReedsShepp/Curve.cpp
    ReedsShepp/CurveSegment.cpp
    ReedsShepp/CircleSegment.cpp
    ReedsShepp/LineSegment.cpp
    ReedsShepp/GeometryHelper.cpp
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
    generic/DistanceTransformation.hpp
    generic/Heuristics.hpp
    generic/DirectNeighborhood.hpp
    generic/NonHolonomicNeighborhood.hpp
    generic/MapManager.hpp
    generic/NodeManager.hpp
    generic/ReedsSheppExpansion.hpp
    evaluation/Renderer.hpp
    evaluation/Util.hpp
    sampling/GoalRegion.cpp
    sampling/RingGoalRegion.cpp
    sampling/PoseListGoalRegion.cpp
    sampling/CentroidRadiusGoalRegion.cpp
    sampling/SamplingPlanner.cpp
)

add_library(LibPath ${files})

target_link_libraries(LibPath LibUtil)

#target_link_libraries(ReedsShepp    LibUtil)

#install(TARGETS ReedsShepp LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

find_package(OpenCV)
if(NOT OpenCV_FOUND)
    message( "OpenCV NOT found, not building test program" )

else(NOT OpenCV_FOUND)

    add_executable (CostPrecomputation
                             ReedsShepp/CostPrecomputation.cpp
                             ReedsShepp/CurveRenderer.cpp)
    target_link_libraries (CostPrecomputation
        LibUtil
        LibPath
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
    )


    add_executable (ReedsSheppCurveTest
                             ReedsShepp/CurveTest.cpp
                             ReedsShepp/CurveRenderer.cpp)
    target_link_libraries (ReedsSheppCurveTest
        LibUtil
        LibPath
        ${OpenCV_LIBS}
    )
    add_executable (Path4wsSimulator
                             common/path4ws_simulator.cpp
                             common/path4ws_sim.cpp
    )
    target_link_libraries (Path4wsSimulator
        LibUtil
        LibPath
        ${OpenCV_LIBS}
    )

set(CMAKE_BUILD_TYPE Debug)
    add_executable (Evaluator
                             evaluation/evaluator.cpp)
    target_link_libraries (Evaluator
        LibUtil
        LibPath
        yaml-cpp
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS})



 #   install(TARGETS ReedsSheppCurveTest RUNTIME DESTINATION bin)
endif(NOT OpenCV_FOUND)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_BAK})

#add_executable(unit_tests ReedsShepp/UnitTest.cpp)

#target_link_libraries (unit_tests
#    LibUtil
#    ReedsShepp
#)



# INSTALL INCLUDE HEADERS
include(../cmake/install_symlink.cmake)
