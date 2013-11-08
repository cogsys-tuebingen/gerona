cmake_minimum_required(VERSION 2.8.3)
project(utils_general)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS ..
    LIBRARIES LibUtil
)


find_package(Eigen REQUIRED)
include_directories(
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)

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
    QtCvImageConverter.cpp
)

target_link_libraries(LibUtil
    ${OpenCV_LIBS}
)

install (TARGETS LibUtil LIBRARY DESTINATION lib)
