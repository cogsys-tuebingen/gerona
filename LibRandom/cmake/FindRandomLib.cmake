# Look for RandomLib
#
# Set
#  RANDOMLIB_FOUND = TRUE
#  RandomLib_INCLUDE_DIRS = /usr/local/include
#  RandomLib_LIBRARIES = /usr/local/lib/libRandom.so
#  RandomLib_LIBRARY_DIRS = /usr/local/lib

find_library (RandomLib_LIBRARIES Random
  PATHS "${CMAKE_INSTALL_PREFIX}/../RandomLib/lib")

if (RandomLib_LIBRARIES)
  get_filename_component (RandomLib_LIBRARY_DIRS
    "${RandomLib_LIBRARIES}" PATH)
  get_filename_component (RANDOMLIB_ROOT_DIR
    "${RandomLib_LIBRARY_DIRS}" PATH)
  set (RandomLib_INCLUDE_DIRS "${RANDOMLIB_ROOT_DIR}/include")
  unset (RANDOMLIB_ROOT_DIR)
  if (NOT EXISTS "${RandomLib_INCLUDE_DIRS}/RandomLib/Config.h")
    unset (RandomLib_INCLUDE_DIRS)
    unset (RandomLib_LIBRARIES)
    unset (RandomLib_LIBRARY_DIRS)
  endif ()
endif ()

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (RandomLib DEFAULT_MSG
  RandomLib_LIBRARY_DIRS RandomLib_LIBRARIES RandomLib_INCLUDE_DIRS)
mark_as_advanced (RandomLib_LIBRARY_DIRS RandomLib_LIBRARIES
  RandomLib_INCLUDE_DIRS)
