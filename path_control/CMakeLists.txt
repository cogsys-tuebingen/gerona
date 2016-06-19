cmake_minimum_required(VERSION 2.8.3)
project(path_control)

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
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
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_control_node
  src/controller_node.cpp
  src/pathcontroller.cpp
)
target_link_libraries(path_control_node
  ${catkin_LIBRARIES}
)
add_dependencies(path_control_node path_msgs_generate_messages_cpp)


add_executable(highlevel_dummy
  src/highlevel_dummy_node.cpp
)
target_link_libraries(highlevel_dummy
  ${catkin_LIBRARIES}
)
add_dependencies(highlevel_dummy path_msgs_generate_messages_cpp)
