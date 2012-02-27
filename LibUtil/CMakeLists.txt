# Utils/LibUtil/CMakeLists.txt

# Load OpenCV cmake module, if wasn't looked for or found yet
find_package( OpenCV )
IF(NOT DEFINED OpenCV_FOUND OR NOT OpenCV_FOUND)
        set(OpenCV_DIR /opt/ros/diamondback/stacks/vision_opencv/opencv2/opencv/share/opencv/)
        find_package( OpenCV REQUIRED )
ENDIF()
message("found OpenCV " ${OpenCV_LIB_DIR})


include_directories(
    ${CMAKE_SOURCE_DIR}/ros/eigen/include
    /opt/ros/diamondback/stacks/geometry/bullet/include/LinearMath
    /opt/ros/electric/stacks/bullet/include/LinearMath
    # Alternative using player Extlib:
    #${CMAKE_SOURCE_DIR}/Extlib # for Eigen
)

add_library (LibUtil SHARED
    ConfigFileReader.cpp
    EggTimer.cpp
    EigenTools.cpp
    LogCollector.cpp
    LowPassFilter.cpp
    MathHelper.cpp
    Misc.cpp
    Stopwatch.cpp
    Strings.cpp
    RamaxxException.cpp
    CvMathHelper.cpp
    CvContourManipulator.cpp
    LaserTo3DTransformation.cpp
    Path.cpp
    runningstat.cpp
    Line2d.cpp
    Line2dSegment.cpp
)

target_link_libraries(LibUtil
    ${OpenCV_LIBS}
    -L${OpenCV_LIB_DIR}
)

install (TARGETS LibUtil LIBRARY DESTINATION lib)
