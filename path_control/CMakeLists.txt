cmake_minimum_required(VERSION 2.8.3)
project(path_control)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  add_definitions(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
  add_definitions(-std=c++0x)
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  actionlib
  path_msgs
  tf
)

catkin_package(
    CATKIN_DEPENDS path_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/controller_node.cpp
  src/pathcontroller.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node path_msgs_generate_messages_cpp)


add_executable(highlevel_dummy
  src/highlevel_dummy_node.cpp
)
target_link_libraries(highlevel_dummy
  ${catkin_LIBRARIES}
)
add_dependencies(highlevel_dummy path_msgs_generate_messages_cpp)

add_executable(direct_path_node
  src/direct_path_node.cpp
)
target_link_libraries(direct_path_node
  ${catkin_LIBRARIES}
)
add_dependencies(direct_path_node path_msgs_generate_messages_cpp)


# this is to list all launch files in qtcreator
file(GLOB_RECURSE ${PROJECT_NAME}_launch_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS launch/*.launch)
add_custom_target(${PROJECT_NAME}_list_all_launch_files SOURCES ${${PROJECT_NAME}_launch_files})

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS highlevel_dummy
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS direct_path_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


