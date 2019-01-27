cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function)

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
  cslibs_navigation_utilities
  cslibs_path_planning
  tf
  laser_geometry
  pcl_ros
  pcl_conversions
  pluginlib
  cv_bridge
  image_transport
  sensor_msgs
  model_based_planner
)

find_package(PCL REQUIRED)
find_package(ALGLIB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

if (${OpenCV_VERSION} MATCHES "3.3.1")
    #########################
    ## OpenCV 3.3.1 Bugfix ##
    #########################
    message(STATUS "Applying OpenCV 3.3.1 Workaround")
    foreach(__cvcomponent ${OpenCV_LIB_COMPONENTS})
        set (__original_cvcomponent ${__cvcomponent})
        if(NOT __cvcomponent MATCHES "^opencv_")
            message(STATUS "Not patching ${__cvcomponent}")
            set(__cvcomponent opencv_${__cvcomponent})
        endif()
        if (TARGET ${__cvcomponent})
            message(STATUS "Patching ${__cvcomponent}")
            set_target_properties(${__cvcomponent} PROPERTIES
              MAP_IMPORTED_CONFIG_DEBUG ""
              MAP_IMPORTED_CONFIG_RELEASE ""
              MAP_IMPORTED_CONFIG_RELWITHDEBINFO ""
              MAP_IMPORTED_CONFIG_MINSIZEREL ""
            )
        endif()
    endforeach(__cvcomponent)
endif()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS Eigen3 ALGLIB
    CATKIN_DEPENDS path_msgs nav_msgs control_msgs cslibs_navigation_utilities
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

    src/factory/abstract_factory.cpp
    src/factory/follower_factory.cpp
    src/factory/controller_factory.cpp
    src/factory/local_planner_factory.cpp
    src/factory/collision_avoider_factory.cpp

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
    src/utils/elevation_map.cpp

    src/collision_avoidance/collision_detector.cpp
    src/collision_avoidance/collision_detector_polygon.cpp
    src/collision_avoidance/collision_detector_ackermann.cpp
    src/collision_avoidance/collision_detector_omnidrive.cpp
    src/collision_avoidance/collision_avoider.cpp

    src/controller/robotcontroller.cpp
    src/controller/robotcontroller_orthogonalexponential.cpp
    src/controller/robotcontroller_ackermann_pid.cpp
    src/controller/robotcontroller_PBR.cpp
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
    src/controller/robotcontroller_potential_field.cpp
    src/controller/robotcontroller_potential_field_TT.cpp
    src/controller/robotcontroller_dynamic_window.cpp
    src/controller/robotcontroller_OFC.cpp
    src/controller/robotcontroller_kinematic_HBZ.cpp
    src/controller/robotcontroller_kinematic_HBZ_TT.cpp
    src/controller/robotcontroller_modelbased.cpp
    src/controller/robotcontroller_ackermann_modelbased.cpp
    src/controller/robotcontroller_ekm.cpp
    src/controller/robotcontroller_ekm_TT.cpp
    src/controller/robotcontroller_velocity_TT.cpp

    src/local_planner/abstract_local_planner.cpp
    src/local_planner/local_planner_null.cpp
    src/local_planner/local_planner_transformer.cpp

    src/local_planner/low_speed/local_planner_astar.cpp

    src/local_planner/high_speed_local_planner.cpp
    src/local_planner/high_speed/local_planner_classic.cpp
    src/local_planner/high_speed/local_planner_static.cpp
    src/local_planner/high_speed/local_planner_reconf.cpp
    src/local_planner/high_speed/local_planner_bfs.cpp
    src/local_planner/high_speed/local_planner_bfs_static.cpp
    src/local_planner/high_speed/local_planner_bfs_reconf.cpp
    src/local_planner/high_speed/local_planner_star.cpp
    src/local_planner/high_speed/local_planner_astar.cpp
    src/local_planner/high_speed/local_planner_star_n.cpp
    src/local_planner/high_speed/local_planner_star_g.cpp
    src/local_planner/high_speed/local_planner_star_static.cpp
    src/local_planner/high_speed/local_planner_star_reconf.cpp
    src/local_planner/high_speed/local_planner_astar_n_static.cpp
    src/local_planner/high_speed/local_planner_astar_g_static.cpp
    src/local_planner/high_speed/local_planner_astar_n_reconf.cpp
    src/local_planner/high_speed/local_planner_astar_g_reconf.cpp
    src/local_planner/model_based/local_planner_model.cpp

    src/local_planner/constraint.cpp
    src/local_planner/constraints/dis2path_constraint.cpp
    src/local_planner/constraints/dis2obst_constraint.cpp

    src/local_planner/scorer.cpp
    src/local_planner/scorers/curvature_scorer.cpp
    src/local_planner/scorers/curvatured_scorer.cpp
    src/local_planner/scorers/dis2pathd_scorer.cpp
    src/local_planner/scorers/dis2pathp_scorer.cpp
    src/local_planner/scorers/level_scorer.cpp
    src/local_planner/scorers/dis2obst_scorer.cpp

    src/supervisor/supervisorchain.cpp
    src/supervisor/pathlookout.cpp
    src/supervisor/obstacletracker.cpp
    src/supervisor/waypointtimeout.cpp
    src/supervisor/distancetopathsupervisor.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${ALGLIB_LIBRARIES} ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${path_msgs_EXPORTED_TARGETS})


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


# this is to list all launch files in qtcreator
file(GLOB_RECURSE ${PROJECT_NAME}_launch_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS launch/*.launch)
add_custom_target(${PROJECT_NAME}_list_all_launch_files SOURCES ${${PROJECT_NAME}_launch_files})
