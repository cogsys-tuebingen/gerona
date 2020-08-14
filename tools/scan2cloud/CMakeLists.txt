cmake_minimum_required(VERSION 2.8.3)
project(scan2cloud)

## Enforce that we use C++14
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  add_definitions(-std=c++14)
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

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



add_executable(simple_scan2cloud_node
    src/simple_scan2cloud_node.cpp
    )
target_link_libraries(simple_scan2cloud_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

install(TARGETS simple_scan2cloud_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})




add_executable(single_scan2cloud_node
    src/single_scan2cloud_node.cpp
    src/${PROJECT_NAME}/scan_processor.cpp
    )
target_link_libraries(single_scan2cloud_node
    ${catkin_LIBRARIES}
    )

install(TARGETS single_scan2cloud_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})




add_executable(multi_scan2cloud_node
    src/multi_scan2cloud_node.cpp
    src/${PROJECT_NAME}/scan_processor.cpp
    )

target_link_libraries(multi_scan2cloud_node
    ${catkin_LIBRARIES}
    )

install(TARGETS multi_scan2cloud_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

