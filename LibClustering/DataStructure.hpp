/*
 * DataStructure.hpp
 *
 *  Created on: Feb 04, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef DATASTRUCTURE_HPP
#define DATASTRUCTURE_HPP

/// PROJECT
#include "Traits.hpp"

/// SYSTEM
#include <vector>
#include <cmath>

namespace lib_clustering {

/**
 * @brief The DataStructure struct is a base class for datastructures that can be used with KMeans
 */
template <class Cluster, typename InputDataType>
struct DataStructure {
    enum { Dimension = Cluster::VectorT::Dimension };

    typedef Cluster ClusterT;
    typedef InputDataType InputDataTypeT;
    typedef typename ClusterT::VectorT VectorT;
    typedef typename VectorT::IndexType VectorIndexType;
    typedef typename VectorT::ValueType VectorValueType;

    /**
     * @brief clear clears all internal storage
     */
    void clear() {
        data.clear();
        clusters.clear();
    }

    /**
     * @brief mergeVector add another vector to the data structure, merge duplicates
     * @param vector
     */
    void mergeVector(VectorT vector) {
        bool already_known = false;
        for(typename std::vector<VectorT>::iterator it = data.begin(); it != data.end(); ++it) {
            if(*it == vector) {
                it->weight += it->weight;
                already_known = true;
                break;
            }
        }

        if(!already_known) {
            data.push_back(vector);
        }
    }

    /**
     * @brief pushVector add another vector to the data structure, <b>doesn't</b> merge duplicates
     * @param vector
     */
    void pushVectorIfValid(VectorT vector) {
        if(vector.weight > 0) {
            data.push_back(vector);
        }
    }

    /**
     * @brief mergeVectorIfValid create and add a vector to the data structure, if the value is greater than zero
     * @param value weight of the vector
     * @param index the vectors indexes
     */
    template <class ValueProvider, class IndexProvider>
    inline void mergeVectorIfValid(const ValueProvider& value, const IndexProvider& index) {
        VectorValueType val = AccessTraits<int, InputDataType>::value(value);
        if(val > 0) {
            mergeVector(AccessTraits<int, InputDataType>:: template readVector<VectorT, IndexProvider, VectorValueType>(index, val));
        }
    }

    /**
     * @brief mergeVectorIfValid
     * @note see mergeVectorIfValid with two parameters
     * @param value object that provides value and indexes
     */
    template <class ValueIndexProvider>
    inline void mergeVectorIfValid(const ValueIndexProvider& x) {
        mergeVectorIfValid(x, x);
    }

    /**
     * @brief assignVectorsToNearestCluster assigns each data point to the cluster whose centroid minimizes the distance
     */
    void assignVectorsToNearestCluster() {
        for(typename std::vector<ClusterT>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
            cluster->members.clear();
        }

        for(typename std::vector<VectorT>::iterator v = data.begin(); v != data.end(); ++v) {

            typename std::vector<ClusterT>::iterator closest_cluster = clusters.end();
            double closest_distance = INFINITY;

            for(typename std::vector<ClusterT>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
                double d = ClusterT::Distance::distance(cluster->centroid, *v);
                if(d < closest_distance) {
                    closest_distance = d;
                    closest_cluster = cluster;
                }
            }

            v->distance = closest_distance;

            closest_cluster->members.push_back(&*v);
        }
    }

public:
    /**
     * @brief data all data points
     */
    std::vector<VectorT> data;

    /**
     * @brief clusters all clusters
     */
    std::vector<ClusterT> clusters;
};

}

#endif // DATASTRUCTURE_HPP
