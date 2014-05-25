cmake_minimum_required(VERSION 2.8.3)
project(roimap)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  utils_general
  utils_robot
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(roimap_node src/RoiMapNode.cpp)
target_link_libraries(roimap_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
