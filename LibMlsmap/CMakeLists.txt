# Utils/LibMlsmap/CMakeLists.txt

#include_directories(
#    /usr/include/eigen2/
#    # Alternative using player Extlib:
#    #${CMAKE_SOURCE_DIR}/Extlib # for Eigen
#)

rosbuild_add_library (LibMlsmap SHARED
    mlsmap.h
    field.h
    vectorcell.cpp
    surface.cpp
    visualizer.cpp
)

rosbuild_add_executable (TestMls TestMls.cpp)

target_link_libraries (TestMls LibMlsmap)

#install (TARGETS LibMlsmap LIBRARY DESTINATION lib)
