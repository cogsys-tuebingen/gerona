cmake_minimum_required(VERSION 2.8.3)
project(odom2tf)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    tf2
    tf2_ros
)

catkin_package()

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node
    src/odom2tf_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

