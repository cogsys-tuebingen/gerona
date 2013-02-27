/*
 * Cluster.hpp
 *
 *  Created on: Feb 05, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef CLUSTER_HPP
#define CLUSTER_HPP

/// PROJECT
#include "Vector.hpp"

/// SYSTEM
#include <vector>

namespace lib_clustering {

/**
 * @brief The Cluster struct represents one cluster
 */
template <typename Vector, class Centroid, class DistanceT>
struct Cluster {
    typedef Vector VectorT;
    typedef typename VectorT::IndexType VectorIndexType;
    typedef typename VectorT::ValueType VectorValueType;
    typedef typename VectorT::UserDataType VectorUserDataType;
    typedef Centroid CentroidT;
    typedef DistanceT Distance;

    /**
     * @brief The MoveResult enum represents the result of a movement action
     */
    enum MoveResult {
        CHANGE, NO_CHANGE
    };

    /**
     * @brief Cluster Constructor
     * @param c initial centroid
     */
    Cluster(const VectorT& c)
        : centroid(c)
    {}

    /**
     * @brief recompute moves the center of each cluster to the centroid of all its members
     * @return <b>NO_CHANGE</b>, iff no significant movement took place, <b>CHANGE</b> otherwise
     */
    MoveResult recompute() {
        if(members.size() == 0) {
            return NO_CHANGE;
        }

        double norm = 0;

        VectorImp<VectorT::Dimension, double, VectorValueType, VectorUserDataType> c(0);
        for(typename std::vector<VectorT*>::iterator member = members.begin(); member != members.end(); ++member) {
            VectorT& v = **member;
            c += (v * v.weight);

            norm += v.weight;
        }

        assert(norm);
        c /= norm;

        VectorT next_centroid = c.template convert<VectorIndexType, VectorValueType>();

        double dist = Distance::distance(next_centroid, centroid);

        centroid = next_centroid;

        return dist > 0.5 ? CHANGE : NO_CHANGE;
    }

public:
    CentroidT centroid;
    std::vector<VectorT*> members;
};

}

#endif // CLUSTER_HPP
