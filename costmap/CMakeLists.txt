cmake_minimum_required(VERSION 2.8.3)
project(costmap)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  utils_general
  utils_robot
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(costmap_node src/CostmapNode.cpp)
target_link_libraries(costmap_node ${catkin_LIBRARIES})
