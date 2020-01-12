cmake_minimum_required(VERSION 2.8.3)
project(odom2tf)

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

