/*
 * Vector.hpp
 *
 *  Created on: Feb 04, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef VECTOR_HPP
#define VECTOR_HPP

/// COMPONENT
#include "Traits.hpp"

/// SYSTEM
#include <assert.h>
#include <iostream>

namespace lib_clustering {

/**
 * @brief The VectorImp struct represents a N dimensional vector of arbitrary value type
 */
template <unsigned N, typename IndexTypeT, typename ValueTypeT, typename UserData>
struct VectorImp {
    /// the vector's dimension
    enum { Dimension = N };

    /// the vector's type
    typedef ValueTypeT ValueType;
    typedef IndexTypeT IndexType;
    typedef UserData UserDataType;

    typedef VectorImp<Dimension, IndexType, ValueType, UserData> VectorType;


    /**
     * @brief VectorImp
     * @param weight
     */
    VectorImp(ValueType weight, UserData* user_data = (UserData*) 0xDEADBEEF)
        : weight(weight), data(user_data) {
        memset(index, 0, sizeof(IndexType) * Dimension);
    }

    /**
     * @brief convert the value type to another type
     * @return
     */
    template <class NewIndexType, class NewValueType>
    VectorImp<Dimension, NewIndexType, NewValueType, UserData> convert() const {
        VectorImp<Dimension, NewIndexType, NewValueType, UserData> result(1, data);

        for(unsigned dim = 0; dim < Dimension; ++dim) {
            result[dim] = index[dim];
        }
        result.weight = weight;
        result.data = data;

        return result;
    }

    /// opertators
    IndexType& operator[](unsigned i) {
        return index[i];
    }

    template <class OtherIndexType, class OtherValueType>
    void operator += (const VectorImp<Dimension, OtherIndexType, OtherValueType, UserData>& o) {
        for(unsigned dim = 0; dim < Dimension; ++dim) {
            index[dim] += o.index[dim];
        }
    }
    void operator /= (double v) {
        assert(v != 0);
        for(unsigned dim = 0; dim < Dimension; ++dim) {
            index[dim] /= v;
        }
    }

    VectorType operator *(double v) const {
        VectorType result(weight, data);
        for(unsigned dim = 0; dim < Dimension; ++dim) {
            result[dim] = index[dim] * v;
        }
        return result;
    }

    bool operator == (const VectorType& v) const {
        for(unsigned dim = 0; dim < Dimension; ++dim) {
            if(index[dim] != v.index[dim]) {
                return false;
            }
        }
        return true;
    }

public:
    IndexType index[Dimension];
    ValueType weight;
    double distance;

    UserData* data;
};

}

#endif // VECTOR_HPP
