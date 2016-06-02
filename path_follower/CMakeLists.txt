cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

set(CMAKE_BUILD_TYPE Debug)
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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   path_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
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

## Declare a cpp library
# add_library(path_follower
#   src/${PROJECT_NAME}/path_follower.cpp
# )

## Declare a cpp executable
# add_executable(path_follower_node src/path_follower_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(path_follower_node path_follower_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(path_follower_node
#   ${catkin_LIBRARIES}
# )

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
  src/utils/extended_kalman_filter.cpp

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
