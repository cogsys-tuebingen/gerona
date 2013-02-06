# Utils/LibUtil/CMakeLists.txt

# Load OpenCV cmake module, if wasn't looked for or found yet
find_package(OpenCV REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/include/utils/LibRandom/include/
)

set (LibClustering
    Clustering.h
    DataStructure.hpp
    DataDense.hpp
    DataSparse.hpp
    Cluster.hpp
    Distance.hpp
    Vector.hpp
    Traits.hpp
    InitializationRandom.hpp
    InitializationPlusPlus.hpp
    KMeans.hpp
)
add_custom_target(LibClustering SOURCES ${LibClustering})

find_package(OpenCV)
if(NOT OpenCV_FOUND)
    message( "OpenCV NOT found, not building test program" )

else(NOT OpenCV_FOUND)
    add_executable (KMeansTest KMeansTest.cpp)
    target_link_libraries (KMeansTest
        ${OpenCV_LIBS}
        Random
    )
endif(NOT OpenCV_FOUND)
