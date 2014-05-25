cmake_minimum_required(VERSION 2.8.3)
project(costmap)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  utils_general
  utils_robot
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(costmap_grow_node src/CostmapNode.cpp)
target_link_libraries(costmap_grow_node ${catkin_LIBRARIES})


add_executable(costmap_gradient_node src/GradientCostmapNode.cpp)
target_link_libraries(costmap_gradient_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
