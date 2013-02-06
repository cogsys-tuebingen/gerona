/*
 * DataDense.hpp
 *
 *  Created on: Feb 04, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef DENSE_HPP
#define DENSE_HPP

/// PROJECT
#include "DataStructure.hpp"

/// SYSTEM
#include <cstdlib>

namespace lib_clustering {

/**
 * @brief The Dense struct implements a sparse dense structure
 */
template <class Cluster, typename InputDataType>
struct Dense : public DataStructure<Cluster, InputDataType> {
    typedef DataStructure<Cluster, InputDataType> DataStruct;

    using DataStruct::Dimension;
    using DataStruct::data;

    typedef typename DataStruct::VectorIndexType VectorIndexType;

    typedef Cluster ClusterT;
    typedef typename Cluster::VectorT VectorT;

    /**
     * @brief prepare takes data in a dense input form and coverts it to the vector representation
     * @param raw the dense data
     * @param limits pairs of limits for each dimension
     */
    void prepare(InputDataType raw,
                 const std::vector<std::pair<VectorIndexType, VectorIndexType> >& limits) {
        DataStruct::clear();

        long entries = 1;
        for(unsigned i = 0; i < limits.size(); ++i) {
            entries *= limits[i].second - limits[i].first;
        }

        typename VectorT::IndexType current_index[Dimension];
        memset(current_index, 0, sizeof(typename VectorT::IndexType) * Dimension);

        for(unsigned idx = 0; idx < entries; ++idx) {
            for(unsigned dim = 0; dim < Dimension; ++dim) {
                current_index[dim]++;
                if(current_index[dim] < limits[dim].second) {
                    break;
                } else {
                    current_index[dim] = limits[dim].first;
                }
            }

            mergeVectorIfValid(raw[idx], current_index);
        }
    }
};

}
#endif // DENSE_HPP
