/*
 * Clustering.h
 *
 *  Created on: Feb 06, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef CLUSTERING_H
#define CLUSTERING_H

/// COMPONENT
#include "ClusteringAlgorithm.hpp"

#include "DataSparse.hpp"
#include "DataDense.hpp"
#include "InitializationPlusPlus.hpp"
#include "InitializationRandom.hpp"
#include "Distance.hpp"

namespace lib_clustering {

/**
 * @brief The KMeans struct wraps the implementation of KMeans to make it easier to use
 */
template <unsigned Dimensions,
         template <class, class, typename> class InitializationMethod,
         template <unsigned> class Distance,
         template <class, typename> class DataType,
         typename InputDataType,
         typename UserData = void,
         typename IndexDataType = int,
         typename WeightDataType = unsigned char>
struct KMeans :
    public ClusteringAlgorithm< KMeansParameter<Dimensions, InitializationMethod, Distance, DataType, InputDataType, UserData, IndexDataType, WeightDataType> >
{
    KMeans(int K)
        : ClusteringAlgorithm< KMeansParameter<Dimensions, InitializationMethod, Distance, DataType, InputDataType, UserData, IndexDataType, WeightDataType> >(K)
    {}
};

}

#endif // CLUSTERING_H
