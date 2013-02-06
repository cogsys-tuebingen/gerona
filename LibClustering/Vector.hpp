/*
 * Vector.hpp
 *
 *  Created on: Feb 04, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef VECTOR_HPP
#define VECTOR_HPP

/// SYSTEM
#include <assert.h>
#include <iostream>

/**
 * @brief The VectorImp struct represents a N dimensional vector of arbitrary value type
 */
template <unsigned N, typename IndexTypeT, typename ValueTypeT>
struct VectorImp {
    /// the vector's dimension
    enum { Dimension = N };

    /// the vector's type
    typedef ValueTypeT ValueType;
    typedef IndexTypeT IndexType;

    typedef VectorImp<Dimension, IndexType, ValueType> VectorType;


    /**
     * @brief VectorImp
     * @param weight
     */
    VectorImp(ValueType weight = 1)
        : weight(weight) {
        memset(index, 0, sizeof(IndexType) * Dimension);
    }

    /**
     * @brief convert the value type to another type
     * @return
     */
    template <class NewIndexType, class NewValueType>
    VectorImp<Dimension, NewIndexType, NewValueType> convert() const {
        VectorImp<Dimension, NewIndexType, NewValueType> result;

        for(unsigned dim = 0; dim < Dimension; ++dim) {
            result[dim] = index[dim];
        }
        result.weight = weight;

        return result;
    }

    /// opertators
    IndexType& operator[](unsigned i) {
        return index[i];
    }

    template <class OtherIndexType, class OtherValueType>
    void operator += (const VectorImp<Dimension, OtherIndexType, OtherValueType>& o) {
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
        VectorType result;
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
};

#endif // VECTOR_HPP
