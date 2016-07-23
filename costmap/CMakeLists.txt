cmake_minimum_required(VERSION 2.8.3)
project(costmap)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  utils_general
  utils_robot
)

find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS nav_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_grow_node src/CostmapNode.cpp)
target_link_libraries(${PROJECT_NAME}_grow_node ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_gradient_node src/GradientCostmapNode.cpp)
target_link_libraries(${PROJECT_NAME}_gradient_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



install(TARGETS ${PROJECT_NAME}_grow_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_gradient_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
