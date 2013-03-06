# src/CMakeLists.txt
cmake_minimum_required (VERSION 2.6)
project (Utils)

include_directories (
    LibUtil
    LibGeneric
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
add_subdirectory (LibLaserProcessing)
add_subdirectory (LibRandom)
add_subdirectory (LibGeneric)
add_subdirectory (LibClustering)
