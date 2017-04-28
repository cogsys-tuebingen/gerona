cmake_minimum_required(VERSION 2.8.3)
project(navigation_launch)

find_package(catkin REQUIRED)

catkin_package(
)

# this is to list all launch files in qtcreator
file(GLOB_RECURSE ${PROJECT_NAME}_launch_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS launch/*.launch)
add_custom_target(list_all_launch_files SOURCES ${${PROJECT_NAME}_launch_files})
