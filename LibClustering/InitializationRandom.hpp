/*
 * InitializationRandom.hpp
 *
 *  Created on: Feb 04, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef INITIALIZATIONRANDOM_HPP
#define INITIALIZATIONRANDOM_HPP

namespace lib_clustering {

/**
 * @brief The RandomInitialization struct implements a random initialization policy
 */
template <class Datatype, class ClusterT, typename InputDataType>
struct RandomInitialization :
        public Datatype
{
    typedef typename ClusterT::VectorT VectorT;
    typedef InputDataType InputDataTypeT;

    enum { Dimension = VectorT::Dimension };

    using Datatype::data;
    using Datatype::clusters;

    typedef typename VectorT::IndexType VectorIndexType;

    /**
     * @brief init initializes the set of clusters to randomly chosen centroids
     * @param K the number of clusters
     * @param limits pairs of limits for each dimension
     */
    void init(unsigned K, const std::vector<std::pair<VectorIndexType, VectorIndexType> >& limits) {
        for(unsigned i = 0; i < K; ++i) {
            clusters.push_back(ClusterT());

            VectorT random_center = random(limits);
            clusters[i].setCentroid(random_center);
        }
    }

private:
    static VectorT random(const std::vector<std::pair<VectorIndexType, VectorIndexType> >& limits) {
        VectorT result;

        for(unsigned dim = 0; dim < Dimension; ++dim) {
            VectorIndexType minv = limits[dim].first;
            VectorIndexType maxv = limits[dim].second;

            int x = minv + rand() / (double) RAND_MAX * (maxv - minv);
            result[dim] = x;
        }

        return result;
    }
};

}

#endif // INITIALIZATIONRANDOM_HPP
