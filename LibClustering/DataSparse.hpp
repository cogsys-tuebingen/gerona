/*
 * DataSparse.hpp
 *
 *  Created on: Feb 04, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef SPARSE_HPP
#define SPARSE_HPP

/// PROJECT
#include "DataStructure.hpp"

/**
 * @brief The SparseNonUnique struct implements a sparse data structure
 */
template <class Cluster, typename InputDataType>
struct SparseNonUnique : public DataStructure<Cluster, InputDataType> {
    typedef DataStructure<Cluster, InputDataType> DataStruct;

    using DataStruct::Dimension;
    using DataStruct::data;

    typedef typename DataStruct::VectorIndexType VectorIndexType;

    typedef Cluster ClusterT;
    typedef typename Cluster::VectorT VectorT;

    /**
     * @brief prepare takes data in a sparse input form and coverts it to the vector representation
     * @param raw the sparse data
     * @param limits pairs of limits for each dimension
     */
    void prepare(const InputDataType& raw, const std::vector<std::pair<VectorIndexType, VectorIndexType> >& limits) {
        DataStruct::clear();

        for(unsigned idx = 0; idx < raw.size(); ++idx) {
            pushVectorIfValid(raw[idx]);
        }
    }
};

#endif // SPARSE_HPP
