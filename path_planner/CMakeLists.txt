cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nav_msgs utils_path roscpp std_msgs tf)


set(CMAKE_BUILD_TYPE Debug)

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

add_executable(path_planner_node src/path_planner_node.cpp)

target_link_libraries(path_planner_node
  ${catkin_LIBRARIES}
)

