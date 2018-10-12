cmake_minimum_required(VERSION 2.8.3)
project(scan2cloud)


#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS "-std=c++11 -march=native ${CMAKE_CXX_FLAGS}")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -ffast-math ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    laser_geometry
    roscpp
    std_msgs
    pcl_ros
    tf
    )

find_package(PCL REQUIRED common filters)

catkin_package(
    CATKIN_DEPENDS laser_geometry roscpp std_msgs tf
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
    )
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
    src/scan2cloud_node.cpp
    src/${PROJECT_NAME}/scan2cloud.cpp
    )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(scan2cloud_node scan2cloud_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


add_executable(single_scan2cloud_node
    src/single_scan2cloud_node.cpp
    )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(scan2cloud_node scan2cloud_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(single_scan2cloud_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

install(TARGETS single_scan2cloud_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


add_executable(multi_scan2cloud_node
    src/multi_scan2cloud_node.cpp
    src/${PROJECT_NAME}/multi_scan2cloud.cpp
    )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(scan2cloud_node scan2cloud_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(multi_scan2cloud_node
    ${catkin_LIBRARIES}
    )

install(TARGETS multi_scan2cloud_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

