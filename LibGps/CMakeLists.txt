
add_library (LibGps
 nmeaparser.cpp
 serialgps.cpp
 ublox.cpp
 ubloxmsg.cpp
 ubloxobservations.cpp
 ecefgk.cpp
 gk.cpp
)

#install (TARGETS LibGps LIBRARY DESTINATION lib)
