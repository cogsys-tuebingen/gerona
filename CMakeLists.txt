# src/CMakeLists.txt
cmake_minimum_required (VERSION 2.6)
project (Utils)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories (
        .
)

add_subdirectory (utils)
