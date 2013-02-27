/*
 * InitializationPlusPlus.hpp
 *
 *  Created on: Feb 05, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef INITIALIZATIONPLUSPLUS_HPP
#define INITIALIZATIONPLUSPLUS_HPP

/// PROJECT
#include <RandomLib/Random.hpp>
#include <RandomLib/RandomSelect.hpp>

/// SYSTEM
#include "assert.h"

namespace lib_clustering {

/**
 * @brief The RandomInitialization struct implements the k-means++ initialization policy
 */
template <class Datatype, class ClusterT, typename InputDataType>
struct PlusPlusInitialization :
    public Datatype {
    typedef typename ClusterT::VectorT VectorT;
    typedef InputDataType InputDataTypeT;

    enum { Dimension = VectorT::Dimension };

    using Datatype::data;
    using Datatype::clusters;
    using Datatype::expectation;

    typedef typename VectorT::IndexType VectorIndexType;

    /**
     * @brief init initializes the set of clusters according to k-means++
     * @param K the number of clusters
     * @param limits pairs of limits for each dimension
     */
    void init(unsigned K, const std::vector<std::pair<VectorIndexType, VectorIndexType> >& limits) {
        // select one random input item
        unsigned first = rand() % data.size();
        clusters.push_back(ClusterT(data[first]));

        std::vector<unsigned> already_selected;
        already_selected.push_back(first);

        for(unsigned i = 1; i < K; ++i) {
            // assign to nearest cluster (computes distance);
            expectation();

            // take the computed distances to centers
            unsigned n = data.size();
            double distances[n];
            for(unsigned j = 0; j < data.size(); ++j) {
                distances[j] = data[j].distance * data[j].distance;
            }

            // choose ith centroid ~(dist(nearest)²)
            RandomLib::RandomSelect<double> selector(distances, distances + n);
            RandomLib::Random r;

            unsigned next;
            do {
                // select a random center until one is taken that is not yet used
                next = selector(r);
            } while(std::find(already_selected.begin(), already_selected.end(), next) != already_selected.end());

            // remember that this one has been taken
            already_selected.push_back(next);

            clusters.push_back(ClusterT(data[next]));
        }
    }
};

}

#endif // INITIALIZATIONPLUSPLUS_HPP
