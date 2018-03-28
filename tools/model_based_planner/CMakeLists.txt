cmake_minimum_required(VERSION 2.8.3)
project(model_based_planner)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native ${CMAKE_CXX_FLAGS}")

#set(CMAKE_BUILD_TYPE Release)

# Set to use AVX code path. If not set SSE is used
add_definitions(-DUSE_AVX2)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function -Wno-deprecated-register)
    add_definitions("-std=c++11 -Ofast -g -rdynamic -march=native -ffast-math -mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mavx -mfma -mavx2 -mfpmath=both")

endif()

if(${CMAKE_BUILD_TYPE} STREQUAL RELWITHDEBINFO)
    add_definitions(-fno-omit-frame-pointer)
endif()


find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp tf
#  DEPENDS Eigen
)

include_directories(include
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


# Find source files
file(GLOB SOURCES src/*.cpp)

file(GLOB HEADERS include/${PROJECT_NAME}/*.h)

# Create shared library
add_library(${PROJECT_NAME} SHARED
    ${HEADERS} # for qtcreator...
    ${SOURCES}
    )


# Install library
#install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install library headers
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME} )

