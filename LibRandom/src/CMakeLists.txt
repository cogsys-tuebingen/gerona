# Build the library...

# Include all the .cpp files in the library.
file (GLOB SOURCES *.cpp)
file (GLOB HEADERS
  ${PROJECT_BINARY_DIR}/include/RandomLib/Config.h
  ../include/RandomLib/*.hpp)

# Define the library and specify whether it is shared or not.
if (RANDOM_SHARED_LIB)
  add_library (Random SHARED ${SOURCES} ${HEADERS})
else ()
  add_library (Random STATIC ${SOURCES} ${HEADERS})
endif ()

# Set the version number on the library
if (WIN32)
  set_target_properties (Random PROPERTIES VERSION "${LIBVERSIONFULL}" )
else ()
  set_target_properties (Random PROPERTIES
    VERSION "${LIBVERSIONFULL}" SOVERSION "${LIBVERSION}" )
endif ()

# Specify where the library is installed, adding it to the export depends
install (TARGETS Random
  EXPORT depends
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
