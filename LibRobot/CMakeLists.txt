# Utils/LibRobot/CMakeLists.txt

#include_directories (
#    ${CMAKE_SOURCE_DIR}/ros/eigen/include
#   ../../Utils/LibUtil/ ../../Player3x/RamaxxDriver/
#)

rosbuild_add_library (LibRobot
    AckermannSteering.cpp
    BoundaryFinder.cpp
    Laser2dScan.cpp
    LaserEnvironment.cpp
    WheelOdometry.cpp
    Costmap.cpp
)

target_link_libraries( LibRobot boost_thread-mt )

#install (TARGETS LibRobot LIBRARY DESTINATION lib)
