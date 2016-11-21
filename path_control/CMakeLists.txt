cmake_minimum_required(VERSION 2.8.3)
project(path_control)

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



install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS highlevel_dummy
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
