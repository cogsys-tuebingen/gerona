# src/CMakeLists.txt

include_directories (
    LibUtil
    LibRobot
)

add_subdirectory (LibUtil)
add_subdirectory (Gnuplot)
add_subdirectory (LibBotHardware)
add_subdirectory (LibOdoCalib)
add_subdirectory (LibGps)
add_subdirectory (LibRobot)
add_subdirectory (LibMlsmap)
add_subdirectory (LibPath)
add_subdirectory (brisk)
