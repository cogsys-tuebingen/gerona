/*
 * Clustering.h
 *
 *  Created on: Feb 06, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef CLUSTERING_H
#define CLUSTERING_H

/// COMPONENT
#include "KMeans.hpp"

#include "DataSparse.hpp"
#include "DataDense.hpp"
#include "InitializationPlusPlus.hpp"
#include "InitializationRandom.hpp"

/**
 * @brief The KMeans struct wraps the implementation of KMeans to make it easier to use
 */
template <unsigned Dimensions,
         template <class, class, typename> class InitializationMethod,
         template <unsigned> class Distance,
         template <class, typename> class DataType,
         typename InputDataType,
         typename IndexDataType = int,
         typename WeightDataType = unsigned char>
struct KMeans :
    public KMeansImp< KMeansParameter<Dimensions, InitializationMethod, Distance, DataType, InputDataType, IndexDataType, WeightDataType> >
{
    KMeans(int K)
        : KMeansImp< KMeansParameter<Dimensions, InitializationMethod, Distance, DataType, InputDataType, IndexDataType, WeightDataType> >(K)
    {}
};

#endif // CLUSTERING_H
