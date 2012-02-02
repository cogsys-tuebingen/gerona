# Utils/LibOdoCalib/CMakeLists.txt

include_directories (
#    ${CMAKE_SOURCE_DIR}/ros/eigen/include
    ../../Utils/LibUtil/
    ../../Utils/LibRobot/
    ../../Utils/Gnuplot
    ../../Utils/LibBotHardware
)

rosbuild_add_library (LibOdoCalib
    BetaEstimator.cpp
    CircleEstimator.cpp
    DriveCircleAction.cpp
    DriveCircleMission.cpp
    DriveStraightAction.cpp
    DriveStraightMission.cpp
    DualAxisCalib.cpp
    HallEstimator.cpp
#    OdoDualPidCtrl.cpp
    MoveAction.cpp
    MoveDistAction.cpp
    MoveTimeAction.cpp
    RobotAction.cpp
    StandSteerAction.cpp
    SteerMission.cpp
    SteerEstimator.cpp
    EncoderMission.cpp
    EncoderEstimator.cpp
    SteerAngleController.cpp
    StraightLineDriver.cpp
    TrajectoryMission.cpp
)

target_link_libraries( LibOdoCalib
    LibRobot
    LibUtil
    ${Boost_LIBRARIES} )

#install (TARGETS LibOdoCalib LIBRARY DESTINATION lib)

