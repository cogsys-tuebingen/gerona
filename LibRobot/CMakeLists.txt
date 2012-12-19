# Utils/LibRobot/CMakeLists.txt

#set(IPP NOTFOUND)
#find_path(IPP ipp.h /opt/intel/ipp/include/)
if(IPP)
    message("found IPP " ${IPP})
    set (IPP_LIB32 NOTFOUND)
    find_path(IPP_LIB32 libippcore.so /opt/intel/ipp/lib/ia32)
    if (IPP_LIB32)
      set(LIBS ippcore ippcv ippi ipps -L/opt/intel/ipp/lib/ia32/)
    else(IPP_LIB32)
      set(LIBS ippcore ippcv ippi ipps -L/opt/intel/ipp/lib/intel64/)
    endif(IPP_LIB32)
    add_definitions(-DIPP)
else(IPP)
    message("didn't find IPP" ${IPP})
    set(LIBS "")
endif(IPP)

# OpenCV
find_package(OpenCV REQUIRED)

include_directories(/opt/intel/ipp/include/)


rosbuild_add_library(LibRobot
    AckermannSteering.cpp
    BoundaryFinder.cpp
    Laser2dScan.cpp
    LaserEnvironment.cpp
    WheelOdometry.cpp
    Costmap.cpp
)


target_link_libraries( LibRobot boost_thread-mt ${OpenCV_LIBRARIES} LibUtil)
