cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

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

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nav_msgs path_msgs cslibs_path_planning roscpp std_msgs tf roslib pcl_ros)

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OMPL)
find_package(SBPL)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp std_msgs path_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Common Library
add_library(${PROJECT_NAME} SHARED src/planner_node.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Generic Search (cslibs_path_planning)
add_executable(${PROJECT_NAME}_node src/path_planner_node.cpp)
add_dependencies(${PROJECT_NAME}_node path_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

### D-Star
#add_executable(dyn_path_planner_node src/dyn_path_planner_node.cpp)
#add_dependencies(dyn_path_planner_node path_msgs_generate_messages_cpp)
#target_link_libraries(dyn_path_planner_node
#  path_planner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
#)
#install(TARGETS dyn_path_planner_node
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


## Static Path "Planner"
add_executable(static_path_node src/static_path_node.cpp)
add_dependencies(static_path_node path_msgs_generate_messages_cpp)
target_link_libraries(static_path_node
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
install(TARGETS static_path_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


add_executable(course_planner_node
    src/course/segment.cpp
    src/course/transition.cpp
    src/course/node.cpp
    src/course/course_map.cpp
    src/course/search.cpp
    src/course/path_builder.cpp
    src/course/analyzer.cpp
    src/course_planner_node.cpp
    src/course_planner.cpp
    src/planner_node.cpp
)
add_dependencies(course_planner_node path_msgs_generate_messages_cpp)
target_link_libraries(course_planner_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
install(TARGETS course_planner_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


add_executable(pub_goal_pose src/pub_goal_pose.cpp)
target_link_libraries(pub_goal_pose
  ${catkin_LIBRARIES}
)
install(TARGETS pub_goal_pose
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})



## OMPL planner
if(${OMPL_FOUND})
    if(${OMPLAPP_LIBRARIES})
        include_directories(${OMPL_INCLUDE_DIRS})

        add_executable(ompl_planner_node src/ompl_planner_node.cpp)
        add_dependencies(ompl_planner_node path_msgs_generate_messages_cpp)

        target_link_libraries(ompl_planner_node
            ${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES}
        )

        install(TARGETS ompl_planner_node
                RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


    else()
        message("not building OMPL planner, your library is missing the OMPLAPP part")
    endif()
else()
    message("not building OMPL planner")
    message("if you want OMPL, see http://ompl.kavrakilab.org/download.html)")
endif()

## SBPL planner
if(${SBPL_FOUND})
    include_directories(${SBPL_INCLUDE_DIRS})

    add_executable(sbpl_planner_node src/sbpl_planner_node.cpp)
    add_dependencies(sbpl_planner_node path_msgs_generate_messages_cpp)

    target_link_libraries(sbpl_planner_node
       ${PROJECT_NAME} ${catkin_LIBRARIES} ${SBPL_LIBRARIES}
    )

    install(TARGETS sbpl_planner_node
            RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


else()
    message("not building SBPL planner")
    message("if you want SBPL, git clone https://github.com/sbpl/sbpl")
endif()

