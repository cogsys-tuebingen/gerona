cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function -Wno-deprecated-register)

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  add_definitions(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
  add_definitions(-std=c++0x)
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  path_msgs
  control_msgs
  nav_msgs
  roscpp
  cslibs_utils
  cslibs_path_planning
  tf
  laser_geometry
  pcl_ros
  pcl_conversions
  pluginlib
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(ALGLIB REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS Eigen3
    CATKIN_DEPENDS path_msgs nav_msgs control_msgs cslibs_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${ALGLIB_INCLUDES}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

file(GLOB_RECURSE INCLUDE_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)

add_library(${PROJECT_NAME} SHARED
    ${INCLUDE_HEADERS}

    src/path_follower_server.cpp
    src/pathfollower.cpp

    src/factory/controller_factory.cpp

    src/utils/parameters.cpp
    src/utils/visualizer.cpp
    src/utils/pose_tracker.cpp
    src/utils/obstacle_cloud.cpp
    src/utils/maptransformer.cpp
    src/utils/cubic_spline_interpolation.cpp
    src/utils/coursepredictor.cpp
    src/utils/path.cpp
    src/utils/movecommand.cpp
    src/utils/path_interpolated.cpp
    src/utils/extended_kalman_filter.cpp

    src/obstacle_avoidance/obstacledetector.cpp
    src/obstacle_avoidance/obstacledetectorpolygon.cpp
    src/obstacle_avoidance/obstacledetectorackermann.cpp
    src/obstacle_avoidance/obstacledetectoromnidrive.cpp
    src/obstacle_avoidance/obstacledetectorpatsy.cpp
    src/obstacle_avoidance/obstacle_avoider.cpp

    src/controller/robotcontroller.cpp
    src/controller/robotcontroller_interpolation.cpp
    src/controller/robotcontroller_ackermann_pid.cpp
    src/controller/robotcontrollertrailer.cpp
    src/controller/path_simple_pid.cpp
    src/controller/path_cascade_pid.cpp
    src/controller/robotcontroller_ICR_CCW.cpp

    src/controller/robotcontroller_ackermann_orthexp.cpp
    src/controller/robotcontroller_ackermann_purepursuit.cpp
    src/controller/robotcontroller_ackermann_inputscaling.cpp
    src/controller/robotcontroller_ackermann_stanley.cpp
    src/controller/robotcontroller_2steer_purepursuit.cpp
    src/controller/robotcontroller_2steer_stanley.cpp
    src/controller/robotcontroller_2steer_inputscaling.cpp
    src/controller/robotcontroller_unicycle_inputscaling.cpp
    src/controller/robotcontroller_differential_orthexp.cpp
    src/controller/robotcontroller_omnidrive_orthexp.cpp
    src/controller/robotcontroller_kinematic_SLP.cpp
    src/controller/robotcontroller_dynamic_SLP.cpp

    src/local_planner/local_planner.cpp
    src/local_planner/local_planner_null.cpp
    src/local_planner/local_planner_implemented.cpp
    src/local_planner/local_planner_classic.cpp
    src/local_planner/local_planner_transformer.cpp
    src/local_planner/local_planner_static.cpp
    src/local_planner/local_planner_reconf.cpp
    src/local_planner/local_planner_bfs.cpp
    src/local_planner/local_planner_bfs_static.cpp
    src/local_planner/local_planner_bfs_reconf.cpp
    src/local_planner/local_planner_star.cpp
    src/local_planner/local_planner_astar.cpp
    src/local_planner/local_planner_thetastar.cpp
    src/local_planner/local_planner_star_n.cpp
    src/local_planner/local_planner_star_g.cpp
    src/local_planner/local_planner_star_static.cpp
    src/local_planner/local_planner_star_reconf.cpp
    src/local_planner/local_planner_astar_n_static.cpp
    src/local_planner/local_planner_astar_g_static.cpp
    src/local_planner/local_planner_astar_n_reconf.cpp
    src/local_planner/local_planner_astar_g_reconf.cpp
    src/local_planner/local_planner_thetastar_n_static.cpp
    src/local_planner/local_planner_thetastar_g_static.cpp
    src/local_planner/local_planner_thetastar_n_reconf.cpp
    src/local_planner/local_planner_thetastar_g_reconf.cpp
    src/local_planner/constraint.cpp
    src/local_planner/dis2path_constraint.cpp
    src/local_planner/dis2obst_constraint.cpp
    src/local_planner/scorer.cpp
    src/local_planner/curvature_scorer.cpp
    src/local_planner/curvatured_scorer.cpp
    src/local_planner/dis2pathd_scorer.cpp
    src/local_planner/dis2pathp_scorer.cpp
    src/local_planner/level_scorer.cpp
    src/local_planner/dis2obst_scorer.cpp

    src/supervisor/supervisorchain.cpp
    src/supervisor/pathlookout.cpp
    src/supervisor/obstacletracker.cpp
    src/supervisor/waypointtimeout.cpp
    src/supervisor/distancetopathsupervisor.cpp
    src/controller/vector_field_histogram.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${ALGLIB_LIBRARIES} ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
)


add_executable(${PROJECT_NAME}_node
  src/follower_node.cpp
)

add_dependencies(${PROJECT_NAME}_node ${path_msgs_EXPORTED_TARGETS} ${control_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

add_executable(pose_goal_remapper
  src/utils/pose_goal_remapper.cpp
)

target_link_libraries(pose_goal_remapper
  ${catkin_LIBRARIES}
)

add_executable(test_output_2_csv
  src/utils/test_output_2_csv.cpp
)

target_link_libraries(test_output_2_csv
  ${catkin_LIBRARIES}
)


#############
## INSTALL ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS pose_goal_remapper
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS test_output_2_csv
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_pathlookout
  test/test_pathlookout.cpp
  include/${PROJECT_NAME}/supervisor/supervisor.h
  src/utils/parameters.cpp
  src/utils/visualizer.cpp
  src/utils/path.cpp
  src/supervisor/pathlookout.cpp
  src/supervisor/obstacletracker.cpp
)
if(TARGET test_pathlookout)
  target_link_libraries(test_pathlookout ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
endif()

find_package(rostest REQUIRED)
add_rostest_gtest(test_obstacletracker
  test/test_obstacletracker.test
  test/test_obstacletracker.cpp
  include/${PROJECT_NAME}/supervisor/supervisor.h
  src/utils/parameters.cpp
  src/utils/visualizer.cpp
  src/utils/path.cpp
  src/supervisor/pathlookout.cpp
  src/supervisor/obstacletracker.cpp
)
if(TARGET test_obstacletracker)
  target_link_libraries(test_obstacletracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
endif()


catkin_add_gtest(test_pidcontroller
  test/test_pidcontroller.cpp
  include/${PROJECT_NAME}/utils/pidcontroller.hpp
)
if(TARGET test_pidcontroller)
  target_link_libraries(test_pidcontroller ${catkin_LIBRARIES})
endif()


catkin_add_gtest(test_path
  test/test_path.cpp
  src/utils/path.cpp
)
if(TARGET test_path)
  target_link_libraries(test_path ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

endif()
