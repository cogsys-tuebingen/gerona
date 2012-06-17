cmake_minimum_required(VERSION 2.4.6)

file(GLOB AGAST_SOURCE_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")

file(GLOB AGAST_HEADER_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/include/agast/*.h")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include/agast)

#build the library dynamic and static versions
add_library(agast SHARED ${AGAST_SOURCE_FILES} ${AGAST_HEADER_FILES})
