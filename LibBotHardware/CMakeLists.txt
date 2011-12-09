# Utils/LibBotHardware/CMakeLists.txt

#include_directories (
#  ../../Utils/LibUtil/  ../../Utils/LibRobot/
#)

rosbuild_add_library (LibBotHardware
    Actuators.cpp 
    Avr32UiHandler.cpp
    Basetime.cpp
    CarmenSerial.c
    Compass.cpp
    RobotSensors.cpp
    Sensor.cpp
    DataListener.cpp 
    Odometry.cpp
    Joystick.cpp
    LaserFx6.cpp
    LogAdapter.cpp
    SerialDevice.cpp
    SpeedController.cpp 
    SteerServo.cpp 
    Voltage.cpp 
    Adns.cpp 
    MouseFlodometry.cpp 
    PidSpeedController.cpp
    Ramaxx.cpp    
    Ranger.cpp
    PtzController.cpp
    RtzController.cpp
    SpacePoint.cpp
    UsbConn.cpp
    WheelEncoder.cpp
    SteerAngleController.cpp
    Avr32Params.cpp

    # still depends on player
    #RobotProxy.cpp
)

target_link_libraries( LibBotHardware  LibUtil LibRobot SDL gsl gslcblas usb hid boost_filesystem)

#install (TARGETS LibBotHardware LIBRARY DESTINATION lib)
