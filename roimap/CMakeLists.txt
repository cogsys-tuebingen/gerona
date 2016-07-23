cmake_minimum_required(VERSION 2.8.3)
project(roimap)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  utils_general
)

find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS nav_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/RoiMapNode.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
