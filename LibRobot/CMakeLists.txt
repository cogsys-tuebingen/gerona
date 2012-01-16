# Utils/LibRobot/CMakeLists.txt

set(IPP NOTFOUND)
find_path(IPP ipp.h /opt/intel/ipp/include/)
if(IPP)
    message("found IPP " ${IPP})
    set(LIBS ippcore ippcv ippi ipps -L/opt/intel/ipp/lib/intel64/)
    add_definitions(-DIPP)
else(IPP)
    message("didn't find IPP" ${IPP})
    set(LIBS "")
endif(IPP)

include_directories(/opt/intel/ipp/include/)


rosbuild_add_library(LibRobot
    AckermannSteering.cpp
    BoundaryFinder.cpp
    Laser2dScan.cpp
    LaserEnvironment.cpp
    WheelOdometry.cpp
    Costmap.cpp
)

target_link_libraries( LibRobot boost_thread-mt ${LIBS})
