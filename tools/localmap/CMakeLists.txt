cmake_minimum_required(VERSION 2.8.3)
project(localmap)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS "-std=c++11 -march=native ${CMAKE_CXX_FLAGS}")

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -ffast-math ${CMAKE_CXX_FLAGS}")

#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -ffast-math ${CMAKE_CXX_FLAGS}")

if(${CMAKE_BUILD_TYPE} STREQUAL RELWITHDEBINFO)
    add_definitions(-fno-omit-frame-pointer)
endif()

#if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
#    add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function -Wno-deprecated-#register)
#    add_definitions("-std=c++11 -Ofast -g -rdynamic -march=native -ffast-math -mmmx -msse -msse2 -msse3 -mssse3 -#msse4.1 -msse4.2 -mpopcnt -mavx -mfma -mavx2 -mfpmath=both")
#
#endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    cv_bridge
    image_transport
    sensor_msgs
    )

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   planeMsg.msg
#   planeArrayMsg.msg
#Message2.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include include/${PROJECT_NAME}
    #  LIBRARIES gpSlam
    #  CATKIN_DEPENDS roscpp tf
    DEPENDS Eigen
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}

    )


file(GLOB HEADERS include/${PROJECT_NAME}/*.h)
file(GLOB SOURCE src/*.cpp)


add_executable(localmap_node
    src/blockmap.cpp
    src/localmap.cpp
    ${HEADERS} # for qtcreator...

    )

target_link_libraries(localmap_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})


install(TARGETS localmap_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


add_executable(localmap_mc_node
    src/blockmap.cpp
    src/localmap_mc.cpp
    ${HEADERS} # for qtcreator...

    )

target_link_libraries(localmap_mc_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})


install(TARGETS localmap_mc_node
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


#target_link_libraries(utils_vision yaml-cpp ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

#add_dependencies(talker beginner_tutorials_generate_messages_cpp)


## Declare a cpp library
# add_library(obstacle_detection
#   src/${PROJECT_NAME}/obstacle_detection.cpp
# )

## Declare a cpp executable
# add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(obstacle_detection_node obstacle_detection_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(obstacle_detection_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS obstacle_detection obstacle_detection_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_obstacle_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
