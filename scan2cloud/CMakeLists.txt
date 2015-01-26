cmake_minimum_required(VERSION 2.8.3)
project(scan2cloud)

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
  INCLUDE_DIRS include
  LIBRARIES scan2cloud
  CATKIN_DEPENDS laser_geometry roscpp std_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/${PROJECT_NAME}	
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(scan2cloud
   src/${PROJECT_NAME}/scan2cloud.cpp
)

link_directories(
        ${PCL_LIBRARY_DIRS}
        ${catkin_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

## link the pcl library against the *.so
target_link_libraries(${PROJECT_NAME}
                        ${PCL_LIBRARIES}
)

## Declare a cpp executable
add_executable(scan2cloud_node src/scan2cloud_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(scan2cloud_node scan2cloud_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(scan2cloud_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )
