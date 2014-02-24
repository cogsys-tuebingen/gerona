# - Try to find the SBPL library
# Once done this will define:
#
# SBPL_FOUND - SBPL was found
# SBPL_INCLUDE_DIRS - The SBPL include directory
# SBPL_LIBRARIES - The SBPL library

include(FindPackageHandleStandardArgs)

# user can set SBPL_PREFIX to specify the prefix path of the SBPL library
# and include directory, either as an environment variable or as an
# argument to cmake ("cmake -DSBPL_PREFIX=...")
if (NOT SBPL_PREFIX)
    set(SBPL_PREFIX $ENV{SBPL_PREFIX})
endif()

# user can set SBPL_LIB_PATH to specify the path for the SBPL library
# (analogous to SBPL_PREFIX)
if (NOT SBPL_LIB_PATH)
    set(SBPL_LIB_PATH $ENV{SBPL_LIB_PATH})
    if (NOT SBPL_LIB_PATH)
        set(SBPL_LIB_PATH ${SBPL_PREFIX})
    endif()
endif()

# user can set SBPL_INCLUDE_PATH to specify the path for the SBPL include
# directory (analogous to SBPL_PREFIX)
if (NOT SBPL_INCLUDE_PATH)
    set(SBPL_INCLUDE_PATH $ENV{SBPL_INCLUDE_PATH})
    if (NOT SBPL_INCLUDE_PATH)
        set(SBPL_INCLUDE_PATH ${SBPL_PREFIX})
    endif()
endif()


# find the SBPL library
find_library(SBPL_LIBRARIES sbpl
    PATHS ${SBPL_LIB_PATH}
        /usr/local/lib
    PATH_SUFFIXES lib build/lib)

# find include path
find_path(SBPL_INCLUDE_DIRS viplanner.h
    PATHS ${SBPL_INCLUDE_PATH}
        /usr/local/include
    PATH_SUFFIXES sbpl/planners)

if (SBPL_INCLUDE_DIRS)
    string(REGEX REPLACE "/sbpl/planners$" "" SBPL_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
else()
    set(SBPL_INCLUDE_DIRS "")
endif()

find_package_handle_standard_args(SBPL DEFAULT_MSG SBPL_LIBRARIES SBPL_INCLUDE_DIRS)

