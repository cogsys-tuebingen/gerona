# Utils/LibUtil/CMakeLists.txt

# Load OpenCV cmake module, if wasn't looked for or found yet
find_package(OpenCV REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/ros/eigen/include
    /opt/ros/electric/stacks/bullet/include/LinearMath
    # Alternative using player Extlib:
    #${CMAKE_SOURCE_DIR}/Extlib # for Eigen
)

add_library (LibUtil SHARED
    CircleEstimator.cpp
    ConfigFileReader.cpp
    EggTimer.cpp
    EigenTools.cpp
    Graphics.cpp
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
)

install (TARGETS LibUtil LIBRARY DESTINATION lib)
