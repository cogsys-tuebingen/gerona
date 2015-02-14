cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nav_msgs path_msgs utils_path roscpp std_msgs tf roslib cmake_modules pcl_ros)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OMPL)
find_package(SBPL)
find_package(OpenCV REQUIRED)

find_package(Eigen QUIET)
if(NOT ${Eigen_FOUND})
  find_package(Eigen3 REQUIRED)
endif()
include_directories(
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
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

add_executable(path_planner_node src/planner_node.cpp src/path_planner_node.cpp)
add_dependencies(path_planner_node path_msgs_generate_messages_cpp)

target_link_libraries(path_planner_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)


add_executable(static_path_node src/planner_node.cpp src/static_path_node.cpp)
add_dependencies(static_path_node path_msgs_generate_messages_cpp)

target_link_libraries(static_path_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

if(${OMPL_FOUND})
    if(${OMPLAPP_LIBRARIES})
        include_directories(${OMPL_INCLUDE_DIRS})

        add_executable(ompl_planner_node src/planner_node.cpp src/ompl_planner_node.cpp)
        add_dependencies(ompl_planner_node path_msgs_generate_messages_cpp)

        target_link_libraries(ompl_planner_node
            ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES}
        )
    else()
        message("not building OMPL planner, your library is missing the OMPLAPP part")
    endif()
else()
    message("not building OMPL planner")
    message("if you want OMPL, see http://ompl.kavrakilab.org/download.html)")
endif()

if(${SBPL_FOUND})
    include_directories(${SBPL_INCLUDE_DIRS})

    add_executable(sbpl_planner_node src/planner_node.cpp src/sbpl_planner_node.cpp)
    add_dependencies(sbpl_planner_node path_msgs_generate_messages_cpp)

    target_link_libraries(sbpl_planner_node
        ${catkin_LIBRARIES} ${SBPL_LIBRARIES}
    )
else()
    message("not building SBPL planner")
    message("if you want SBPL, git clone https://github.com/sbpl/sbpl")
endif()

