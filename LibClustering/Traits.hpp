/*
 * Traits.hpp
 *
 *  Created on: Feb 05, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef TRAITS_HPP
#define TRAITS_HPP

namespace lib_clustering {

/**
 * @brief The AccessTraits struct maps input data to internal data
 */
template <typename IndexType, class T>
struct AccessTraits {
    /**
     * @brief index accesses the ith index component of the input data
     * @param in input data
     * @param index
     * @return the ith index component of the input data
     */
    template <typename IndexProvider>
    static IndexType index(const IndexProvider& in, unsigned index) {
        return in[index];
    }

    /**
     * @brief value accesses the value of the input data
     * @param in input data
     * @return the value of the input data
     */
    template <typename ValueProvider>
    static ValueProvider value(const ValueProvider& in) {
        return in;
    }

    /**
     * @brief readVector reads a full input vector (index + value)
     * @param in indexes
     * @param val value
     * @return the combined vector
     */
    template <class VectorT, class IndexProvider, class Type>
    static VectorT readVector(const IndexProvider& in, Type& val) {
        VectorT vector(val);
        copy(vector, in);
        return vector;
    }
    template <class VectorT, typename IndexProvider>
    /**
     * @brief readVector reads an empty vector (index only)
     * @param in indexes
     * @return the combined vector with default value
     */
    static VectorT readEmptyVector(const IndexProvider& in) {
        VectorT vector;
        copy(vector, in);
        return vector;
    }

private:
    template <class VectorT, typename IndexProvider>
    static void copy(VectorT& vector, const IndexProvider& in) {
        for(unsigned dim = 0; dim < VectorT::Dimension; ++dim) {
            vector[dim] = AccessTraits<IndexType, T>::index(in, dim);
        }
    }
};

}

#endif // TRAITS_HPP
