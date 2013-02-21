# Utils/LibUtil/CMakeLists.txt

# Load OpenCV cmake module, if wasn't looked for or found yet
find_package(OpenCV REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/include/utils/LibRandom/include/
)

set (LibClustering
    Cluster.hpp
    Clustering.h
    ClusteringAlgorithm.hpp
    DataStructure.hpp
    DataDense.hpp
    DataSparse.hpp
    Distance.hpp
    InitializationRandom.hpp
    InitializationPlusPlus.hpp
    Traits.hpp
    Vector.hpp
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
        LibUtil
    )

    add_executable (EMMaxTest EMMaxTest.cpp)
    target_link_libraries (EMMaxTest
        ${OpenCV_LIBS}
        Random
        LibUtil
    )
endif(NOT OpenCV_FOUND)
