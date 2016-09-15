cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # -std=gnu++0x

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  path_msgs
  control_msgs
  nav_msgs
  roscpp
  utils_general
  utils_robot
  tf
  cmake_modules
  laser_geometry
  pcl_ros
  pcl_conversions
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES path_follower
#  CATKIN_DEPENDS actionlib path_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

find_package(Eigen QUIET)
if(NOT ${Eigen_FOUND})
  find_package(Eigen3 REQUIRED)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


file(GLOB_RECURSE INCLUDE_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)


add_executable(path_follower_node
  ${INCLUDE_HEADERS}
  src/follower_node.cpp
  src/pathfollower.cpp

  src/utils/parameters.cpp
  src/utils/visualizer.cpp
  src/utils/maptransformer.cpp
  src/utils/cubic_spline_interpolation.cpp
  src/utils/coursepredictor.cpp
  src/utils/path.cpp
  src/utils/movecommand.cpp
  src/utils/path_interpolated.cpp

  src/obstacle_avoidance/obstacledetector.cpp
  src/obstacle_avoidance/obstacledetectorpolygon.cpp
  src/obstacle_avoidance/obstacledetectorackermann.cpp
  src/obstacle_avoidance/obstacledetectoromnidrive.cpp
  src/obstacle_avoidance/obstacledetectorpatsy.cpp

  src/controller/robotcontroller.cpp
  src/controller/robotcontroller_interpolation.cpp
  src/controller/robotcontroller_ackermann_pid.cpp
  src/controller/robotcontrollertrailer.cpp
  src/controller/path_simple_pid.cpp
  src/controller/path_cascade_pid.cpp

  src/legacy/robotcontroller_ackermann_orthexp.cpp
  src/legacy/robotcontroller_ackermann_purepursuit.cpp
  src/legacy/robotcontroller_ackermann_kinematic.cpp
  src/legacy/robotcontroller_ackermann_stanley.cpp
  src/legacy/robotcontroller_4ws_purepursuit.cpp
  src/legacy/robotcontroller_4ws_stanley.cpp
  src/legacy/robotcontroller_4ws_inputscaling.cpp
  src/legacy/robotcontroller_unicycle_inputscaling.cpp
  src/legacy/robotcontroller_differential_orthexp.cpp
  src/legacy/robotcontroller_omnidrive_orthexp.cpp
  src/legacy/robotcontroller_kinematic_SLP.cpp
  src/legacy/robotcontroller_dynamic_SLP.cpp
  src/legacy/robotcontroller_kinematic_SSG.cpp

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
  src/local_planner/dis2pathd_scorer.cpp
  src/local_planner/dis2pathp_scorer.cpp
  src/local_planner/level_scorer.cpp
  src/local_planner/dis2obst_scorer.cpp

  src/supervisor/supervisorchain.cpp
  src/supervisor/pathlookout.cpp
  src/supervisor/obstacletracker.cpp
  src/supervisor/waypointtimeout.cpp
  src/supervisor/distancetopathsupervisor.cpp
  src/legacy/vector_field_histogram.cpp

  src/alglib/interpolation.cpp
  src/alglib/alglibinternal.cpp
  src/alglib/alglibmisc.cpp
  src/alglib/ap.cpp
  src/alglib/dataanalysis.cpp
  src/alglib/diffequations.cpp
  src/alglib/fasttransforms.cpp
  src/alglib/integration.cpp
  src/alglib/linalg.cpp
  src/alglib/optimization.cpp
  src/alglib/solvers.cpp
  src/alglib/specialfunctions.cpp
  src/alglib/statistics.cpp
)

add_dependencies(path_follower_node path_msgs_generate_messages_cpp control_msgs_generate_messages_cpp)
target_link_libraries(path_follower_node
  ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
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
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS path_follower path_follower_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_pathlookout
  test/test_pathlookout.cpp
  include/path_follower/supervisor/supervisor.h
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
  include/path_follower/supervisor/supervisor.h
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
  include/path_follower/utils/pidcontroller.hpp
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

endif(CATKIN_ENABLE_TESTING)
