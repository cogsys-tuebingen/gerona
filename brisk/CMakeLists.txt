cmake_minimum_required(VERSION 2.4.6)


FIND_PACKAGE(OpenCV REQUIRED)
include_directories(${OpenCV2_INCLUDE_DIRS})

# Set default compile flags for GCC
add_definitions( -D_REENTRANT -DMX_COMPAT_32 -DMATLAB_MEX_FILE -Wall  )
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions( -fPIC -msse2 -mssse3 -march=nocona -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long )
else(CMAKE_COMPILER_IS_GNUCXX)
	  add_definitions( -fno-strict-aliasing )
endif(CMAKE_COMPILER_IS_GNUCXX)

#agast
add_subdirectory (${PROJECT_SOURCE_DIR}/include/utils/brisk/thirdparty/agast) 
include_directories (${PROJECT_SOURCE_DIR}/include/utils/brisk/thirdparty/agast/include) 
include_directories (${PROJECT_SOURCE_DIR}/include/utils/brisk/include) 

add_library(brisk SHARED src/brisk.cpp)

install (TARGETS brisk LIBRARY DESTINATION lib)
