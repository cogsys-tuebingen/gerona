cmake_minimum_required (VERSION 2.6)
set(CMAKE_LEGACY_CYGWIN_WIN32 0) # Remove when CMake >= 2.8.4 is required
project (RandomLib)

set (RandomLib_VERSION_MAJOR 1)
set (RandomLib_VERSION_MINOR 6)
set (RandomLib_VERSION "${RandomLib_VERSION_MAJOR}.${RandomLib_VERSION_MINOR}")
set (PACKAGE_VERSION ${RandomLib_VERSION})
set (LIBVERSION 1)
set (LIBVERSIONFULL 1.0.6)

# User-settable variables

# (1) COMMON_INSTALL_PATH governs the installation convention.  If it
# is on ON (the Linux default), the installation to a common
# directory, e.g., /usr/local.  If it is OFF (the Windows default),
# the installation directory contains the package name, e.g.,
# c:/pkg/RandomLib-1.6.  The installation directories for the
# documentation and cmake configuration all depend on the variable
# with deeper paths relative to CMAKE_INSTALL_PREFIX being used when
# it's ON.

if (WIN32)
  option (COMMON_INSTALL_PATH "Use a common installation path for packages" OFF)
else ()
  option (COMMON_INSTALL_PATH "Use a common installation path for packages" ON)
endif ()

# (2)  PACKAGE_PATH and INSTALL_PATH govern the find_package search
# path and the installation directory.  (find_package is not used by
# RandomLib since it doesn't depend on other packages.  However
# PACKAGE_PATH is used here for uniformity with other packages which
# adopt the same conventions.)

# If PACKAGE_PATH is defined, it is prepended to CMAKE_PREFIX_PATH.
#
# If INSTALL_PATH is not specified but PACKAGE_PATH is, then
# INSTALL_PATH is set to
#   ${PACKAGE_PATH}, if COMMON_INSTALL_PATH is ON;
#   ${PACKAGE_PATH}/${PROJECT_NAME}-${PACKAGE_VERSION}, otherwise.
#
# If INSTALL_PATH is now defined, then set CMAKE_INSTALL_PREFIX to
# INSTALL_PATH.
#
# Typically, only PACKAGE_PATH needs to be specified, e.g.,
# cmake -D PACKAGE_PATH=/opt .. (on Linux)
#   => CMAKE_PREFIX_PATH=/opt   CMAKE_INSTALL_PREFIX=/opt
# cmake -D PACKAGE_PATH=C:/pkg .. (on Windows)
#   => CMAKE_PREFIX_PATH=C:/pkg CMAKE_INSTALL_PREFIX=C:/pkg/RandomLib-1.6

if (DEFINED PACKAGE_PATH)
  set (CMAKE_PREFIX_PATH ${PACKAGE_PATH} ${CMAKE_PREFIX_PATH})
  message (STATUS "CMAKE_PREFIX_PATH set to ${CMAKE_PREFIX_PATH}")
elseif (DEFINED CMAKE_PREFIX_PATH
    AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  # Backwards compatibility (support the mechanism advertised in
  # version 1.5).
  list (GET CMAKE_PREFIX_PATH 0 PACKAGE_PATH)
endif ()

if (NOT DEFINED INSTALL_PATH AND DEFINED PACKAGE_PATH)
  if (COMMON_INSTALL_PATH)
    set (INSTALL_PATH ${PACKAGE_PATH} CACHE PATH "Installation directory" FORCE)
  else ()
    set (INSTALL_PATH ${PACKAGE_PATH}/${PROJECT_NAME}-${PACKAGE_VERSION}
      CACHE PATH "Installation directory" FORCE)
  endif ()
endif ()
if (DEFINED INSTALL_PATH)
  set (CMAKE_INSTALL_PREFIX ${INSTALL_PATH})
endif ()
message (STATUS "CMAKE_INSTALL_PREFIX set to ${CMAKE_INSTALL_PREFIX}")

# (3) Build as a shared library?  On Windows systems, this is typically
# more trouble than it's worth.
if (WIN32 OR CYGWIN)
  option (RANDOM_SHARED_LIB "Build RandomLib as a shared library" OFF)
else ()
  option (RANDOM_SHARED_LIB "Build RandomLib as a shared library" ON)
endif ()

# (4) Create the documentation?  This depends on whether doxygen can be
# found.  If this is OFF, then links will be provided to the online
# documentation on Sourceforge.
if (WIN32)
  option (RANDOMLIB_DOCUMENTATION
    "Use doxygen to create the documentation" OFF)
else ()
  option (RANDOMLIB_DOCUMENTATION
    "Use doxygen to create the documentation" ON)
endif ()

# (5) By default, cmake looks for hardware support of vector operations.
# This option allows you to disable this.
option (DISABLE_VECTOR_OPTIMIZATIONS "Do not look for SSE2 or AltiVec support"
  OFF)

set(DISABLE_VECTOR_OPTIMIZATIONS ON)

# (6) By default, cmake looks for boost which is used by some of the
# examples. This option allows you to disable this.
option (DISABLE_BOOST "Do not look for boost libraries" OFF)

# Finding other packages.  (These are only need for the examples.  The
# library does not depend on them.)

# On Windows we use boost's static libraries.
if (NOT DISABLE_BOOST)
  if (WIN32)
    set (Boost_USE_STATIC_LIBS ON)
  endif ()
  find_package (Boost COMPONENTS serialization date_time)
endif ()

# Optionally use OpenMP in RandomParallel.
find_package (OpenMP)

# The debug version of the library is called Random_d.
#set (CMAKE_DEBUG_POSTFIX _d)

# Look for vector support SSE2 for Intel chips and AltiVec for PowerPC.
include (CheckIncludeFileCXX)
if (DISABLE_VECTOR_OPTIMIZATIONS)
  set (VECTOR_FLAGS "")
  set (HAVE_SSE2 OFF)
  set (HAVE_ALTIVEC OFF)
else ()
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "amd64" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
    check_include_file_cxx ("emmintrin.h" HAVE_SSE2)
    if (HAVE_SSE2)
      if (WIN32)
        set (VECTOR_FLAGS "/arch:SSE2")
      else ()
        set (VECTOR_FLAGS "-msse2")
      endif ()
    else ()
      set (VECTOR_FLAGS "")
    endif ()
  elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "powerpc")
    check_type_size ("vector unsigned" VECTOR_UNSIGNED BUILTIN_TYPES_ONLY)
    if (HAVE_VECTOR_UNSIGNED)
      set (VECTOR_FLAGS "-maltivec")
      set (HAVE_ALTIVEC ON)
    else ()
      set (VECTOR_FLAGS "")
      set (HAVE_ALTIVEC OFF)
    endif ()
  endif ()
endif ()

# Optionally compile MPFR example.  This requires MPFR 3.0 or later;
# the check for the version currently occurs in the MPFR source files.
check_include_file_cxx ("mpfr.h" HAVE_MPFR)
if (HAVE_MPFR)
  find_library (MPFR_LIBRARIES mpfr)
  find_library (GMP_LIBRARIES gmp)
  if (NOT (MPFR_LIBRARIES AND GMP_LIBRARIES))
    set (HAVE_MPFR OFF)
  endif ()
endif ()

# Determine system properties
include (CheckTypeSize)
check_type_size ("long double" LONG_DOUBLE BUILTIN_TYPES_ONLY)

# Create a Config.h to expose system information to the compiler
configure_file (
    include/RandomLib/Config.h.in
    include/RandomLib/Config.h )

# The documentation depends on doxygen.  Need version 1.8.1.2 or later
# because of use of &minus; etc.
if (RANDOMLIB_DOCUMENTATION)
  set (DOXYGEN_SKIP_DOT ON)
  find_package (Doxygen 1.8.1.2)
  if (DOXYGEN_FOUND)
    execute_process (COMMAND ${DOXYGEN_EXECUTABLE} --version
      OUTPUT_VARIABLE DOXYGEN_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (DOXYGEN_VERSION VERSION_LESS 1.8.1.2)
      set (DOXYGEN_FOUND FALSE)
      message (STATUS "Doxygen version found, ${DOXYGEN_VERSION}, is too old")
    endif ()
  endif ()
endif ()

# Set a default build type for single-configuration cmake generators if
# no build type is set.
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   set (CMAKE_BUILD_TYPE Release)
endif ()

# On non-Windows machine, make the compiler more picky.  Also unrool
# loops in optimizing build types.
if (NOT WIN32)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VECTOR_FLAGS} -Wall -Wextra")
  set (CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -funroll-loops")
  set (CMAKE_CXX_FLAGS_RELEASE
    "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops")
endif ()

# The list of examples to build.  Don't include MPFRExample here since
# it doesn't get linked with RandomLib.  This is treated specially in
# examples/CMakeLists.txt
set (EXAMPLES
  RandomExample RandomTime RandomThread RandomSave
  RandomExact RandomLambda RandomCoverage)

# Set the include directories.  Look in ${PROJECT_BINARY_DIR}/include
# first because that's where Config.h will be
include_directories ("${PROJECT_BINARY_DIR}/include" include)

# The list of subdirectories to process
add_subdirectory (src)
add_subdirectory (include/RandomLib)
#add_subdirectory (examples)
#add_subdirectory (doc)
add_subdirectory (cmake)

# Packaging support; we deal with
# (1) a source distribution: cmake make a tar.gz file and the zip file
# is created from this.
# (2) a binary distribution: code is included for Linux, Apple, and
# Windows, but only the Windows distribution has been exercised.

# Need to ensure that system dlls get included in a binary distribution
if (NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
  # Visual Studio Express does include redistributable components so
  # squelch the warning.
  set (CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
endif ()
include (InstallRequiredSystemLibraries)

# The configuration of CPack is via variable that need to be set before
# the include (CPack).
set (CPACK_PACKAGE_VERSION_MAJOR ${RandomLib_VERSION_MAJOR})
set (CPACK_PACKAGE_VERSION_MINOR ${RandomLib_VERSION_MINOR})
set (CPACK_PACKAGE_VERSION_PATCH 0)
set (CPACK_PACKAGE_CONTACT charles@karney.com)
set (CPACK_PACKAGE_VENDOR "RandomLib")
set (CPACK_PACKAGE_DESCRIPTION_SUMMARY
  "Random library and documentation")
# The list of files to be excluded from the source distribution.
set (CPACK_SOURCE_IGNORE_FILES
  "#"
  "~\$"
  "/\\\\.git"
  "${PROJECT_SOURCE_DIR}/BUILD"
  "${PROJECT_SOURCE_DIR}/tests/"
  "${PROJECT_SOURCE_DIR}/distrib/"
  "${PROJECT_SOURCE_DIR}/[^/]*\\\\.html\$"
  "${PROJECT_SOURCE_DIR}/makefile-admin\$"
  "\\\\.eps\$" )
set (CPACK_SOURCE_GENERATOR TGZ)

set (CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/LICENSE.txt)
set (CPACK_PACKAGE_INSTALL_DIRECTORY
  "${PROJECT_NAME}-${PACKAGE_VERSION}")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY}")

if (WIN32)
  # The Windows binary packager is NSIS.  Set the necessary variables
  # for this.
  set (CPACK_NSIS_CONTACT "charles@karney.com")
  set (CPACK_NSIS_URL_INFO_ABOUT "http://randomlib.sf.net")
  set (CPACK_NSIS_HELP_LINK "mailto:charles@karney.com")
  if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    # Hardcode the prefix for Visual Studio 10
    set (CPACK_NSIS_INSTALL_ROOT "C:\\\\pkg-vc10-x64")
    set (CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY}-win64")
    set (CPACK_NSIS_PACKAGE_NAME
      "${PROJECT_NAME} x64 ${PACKAGE_VERSION}")
    set (CPACK_PACKAGE_INSTALL_REGISTRY_KEY
      "${PROJECT_NAME}-x64-${PACKAGE_VERSION}")
  else ()
    # Hardcode the prefix for Visual Studio 10
    set (CPACK_NSIS_INSTALL_ROOT "C:\\\\pkg-vc10")
    set (CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY}-win32")
    set (CPACK_NSIS_PACKAGE_NAME
      "${PROJECT_NAME} ${PACKAGE_VERSION}")
    set (CPACK_PACKAGE_INSTALL_REGISTRY_KEY
      "${PROJECT_NAME}-${PACKAGE_VERSION}")
  endif ()
  set (CPACK_NSIS_DISPLAY_NAME ${CPACK_NSIS_PACKAGE_NAME})
  set (CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
  CreateShortCut \\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\Library Documentation.lnk\\\" \\\"$INSTDIR\\\\share\\\\doc\\\\RandomLib\\\\html\\\\index.html\\\"
")
  set (CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "
  !insertmacro MUI_STARTMENU_GETFOLDER Application $MUI_TEMP
  Delete \\\"$SMPROGRAMS\\\\$MUI_TEMP\\\\Library Documentation.lnk\\\"
")
  set (CPACK_NSIS_MODIFY_PATH ON)
elseif (APPLE)
  # Not tested
  set (CPACK_GENERATOR DMG)
  set (CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY}-darwin")
else ()
  # Not tested
  set (CPACK_GENERATOR TGZ)
endif ()

include (CPack)

# Another maintainer-specific target is building the source distribution
# via the target dist.  This calls package_source to make a tar.gz file.
# However this needs to be touched up to support the vanilla Makefiles
# provided with RandomLib.  This entails
# (1) creating Makefile (which includes Makefile.mk);
# (2) creating a bare-bones Config.h (with just the version information);
# (3) making sure that make thinks the generated documentation files are
# up-to-date.
# Then a new tar.gz file and zip file are created.  To avoid potential
# problems with directory permissions, tar and zip are told only to
# archive the files.
if (1)
  add_custom_target (dist
    COMMAND ${CMAKE_MAKE_PROGRAM} package_source
    COMMAND
      cd _CPack_Packages/Linux-Source/TGZ/${CPACK_SOURCE_PACKAGE_FILE_NAME} &&
      echo include Makefile.mk > Makefile &&
      sed -e "s/Unconfigured/${PACKAGE_VERSION}/"
      include/RandomLib/Config.h > include/RandomLib/Config.h.new &&
      mv include/RandomLib/Config.h.new include/RandomLib/Config.h
    COMMAND
      cd _CPack_Packages/Linux-Source/TGZ/${CPACK_SOURCE_PACKAGE_FILE_NAME} &&
      chmod -R g-w .
    COMMAND
      cd _CPack_Packages/Linux-Source/TGZ &&
      find ${CPACK_SOURCE_PACKAGE_FILE_NAME} -type f |
      tar cfzT ${CMAKE_BINARY_DIR}/${CPACK_SOURCE_PACKAGE_FILE_NAME}.tar.gz -
    COMMAND
      rm -f ${CMAKE_BINARY_DIR}/${CPACK_SOURCE_PACKAGE_FILE_NAME}.zip &&
      rsync -a --delete
      _CPack_Packages/Linux-Source/TGZ/${CPACK_SOURCE_PACKAGE_FILE_NAME}
      _CPack_Packages/Linux-Source/TGZ.DOS/ &&
      cd _CPack_Packages/Linux-Source/TGZ.DOS &&
      find . -type f |
      egrep  '/\(doxyfile.in|MANIFEST.in|NEWS|AUTHORS|INSTALL|.*\\.\(cpp|hpp|h.in|txt|cmake.in|cmake|h\)\)$$' |
      xargs unix2dos -q -k &&
      find ${CPACK_SOURCE_PACKAGE_FILE_NAME} -type f |
      zip -q ${CMAKE_BINARY_DIR}/${CPACK_SOURCE_PACKAGE_FILE_NAME}.zip -@
  )
endif ()

# Add a test target; the tests are in tools.
enable_testing ()
