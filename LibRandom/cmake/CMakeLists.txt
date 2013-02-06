# config file support for find_package(RandomLib).  This needs to
# deal with two environments: (1) finding the build tree and (2)
# finding the install tree.  randomlib-config.cmake detects which
# situation it is handing by looking at @RANDOMLIB_ROOT_DIR@.  If
# this is an absolute path, it's in the build tree; otherwise, it's in the
# install tree.  (Note that the whole install tree can be relocated.)

# randomlib-config.cmake for the build tree
set (RANDOMLIB_ROOT_DIR "${PROJECT_BINARY_DIR}")
set (RandomLib_INCLUDE_DIRS
  "${PROJECT_BINARY_DIR}/include" "${PROJECT_SOURCE_DIR}/include" )
configure_file (randomlib-config.cmake.in
  "${PROJECT_BINARY_DIR}/randomlib-config.cmake" @ONLY)
configure_file (randomlib-config-version.cmake.in
  "${PROJECT_BINARY_DIR}/randomlib-config-version.cmake" @ONLY)
export (TARGETS Random ${TOOLS}
  FILE "${PROJECT_BINARY_DIR}/randomlib-depends.cmake")

# randomlib-config.cmake for the install tree.  It's installed in
# ${INSTALL_CMAKE_DIR} and @RANDOMLIB_ROOT_DIR@ is the relative
# path to the root from there.  (Note that the whole install tree can
# be relocated.)
if (COMMON_INSTALL_PATH)
  set (INSTALL_CMAKE_DIR "share/cmake/${PROJECT_NAME}-${PACKAGE_VERSION}")
  set (RANDOMLIB_ROOT_DIR "../../..")
else ()
  set (INSTALL_CMAKE_DIR "cmake")
  set (RANDOMLIB_ROOT_DIR "..")
endif ()
# @RandomLib_INCLUDE_DIRS@ is not used in the install tree; reset
# it to prevent the source and build paths appearing in the installed
# config files
set (RandomLib_INCLUDE_DIRS IGNORE)
configure_file (randomlib-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/randomlib-config.cmake @ONLY)
configure_file (randomlib-config-version.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/randomlib-config-version.cmake @ONLY)
install (FILES
  "${CMAKE_CURRENT_BINARY_DIR}/randomlib-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/randomlib-config-version.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}")
# Make information about the cmake targets (the library and the tools)
# available.
install (EXPORT depends
  FILE randomlib-depends.cmake
  DESTINATION "${INSTALL_CMAKE_DIR}")
