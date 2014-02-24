cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nav_msgs utils_path roscpp std_msgs tf roslib)

set(CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OMPL)
find_package(SBPL)

find_package(Eigen REQUIRED)
include_directories(
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp std_msgs
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_planner_node src/planner_node.cpp src/path_planner_node.cpp)

target_link_libraries(path_planner_node
  ${catkin_LIBRARIES}
)

if(${OMPL_FOUND})
    include_directories(${OMPL_INCLUDE_DIRS})

    add_executable(ompl_planner_node src/planner_node.cpp src/ompl_planner_node.cpp)

    target_link_libraries(ompl_planner_node
        ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES}
    )
else()
    message("not building OMPL planner")
endif()

if(${SBPL_FOUND})
    include_directories(${SBPL_INCLUDE_DIRS})

    add_executable(sbpl_planner_node src/planner_node.cpp src/sbpl_planner_node.cpp)

    target_link_libraries(sbpl_planner_node
        ${catkin_LIBRARIES} ${SBPL_LIBRARIES}
    )
else()
    message("not building SBPL planner")
endif()

