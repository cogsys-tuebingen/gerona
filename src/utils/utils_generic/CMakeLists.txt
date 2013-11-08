cmake_minimum_required(VERSION 2.8.3)
project(utils_generic)

find_package(catkin REQUIRED COMPONENTS utils_general)

catkin_package(
    INCLUDE_DIRS ..
    LIBRARIES LibGeneric
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set (LibGeneric
    Intermission.hpp
    Utils.hpp
)
add_custom_target(LibGeneric SOURCES ${LibGeneric})
