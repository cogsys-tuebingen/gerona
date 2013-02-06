/*
 * Distance.hpp
 *
 *  Created on: Feb 04, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef DISTANCE_HPP
#define DISTANCE_HPP

/// PROJECT
#include "Vector.hpp"

/// SYSTEM
#include <cmath>

namespace lib_clustering {

/**
 * @brief The EuclideanDistance struct implements a distance measure using the euclidean distance
 */
template <unsigned Dimensions>
struct EuclideanDistance {
    template <typename TypeAI, typename TypeAV, typename TypeBI, typename TypeBV>

    /**
     * @brief distance between two vectors
     * @param a
     * @param b
     * @return euclidean distance between a and b
     */
    static double distance(const VectorImp<Dimensions, TypeAI, TypeAV>& a, const VectorImp<Dimensions, TypeBI, TypeBV>& b) {
        double distance_squared = 0;
        for(unsigned dim = 0; dim < Dimensions; ++dim) {
            TypeAI delta = (a.index[dim] - b.index[dim]);
            distance_squared += delta * delta;
        }

        return std::sqrt(distance_squared);
    }
};

}

#endif // DISTANCE_HPP
