/*
 * Utils.hpp
 *
 *  Created on: Feb 06, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef UTILS_HPP
#define UTILS_HPP

namespace generic {

/**
 * @brief The Int2Type struct creates a unique type for an integer.
 *        Useful for typematching integer template parameters
 */
template <int v>
struct Int2Type
{
    enum { value = v };
};


/**
 * @brief The Type2Type struct allows to convert a type to a new unique type.
 *        This behaviour can be used to convert partial specialization to overloaded functions.
 *        Member functions can't normaly be partially specialized, so you have to use overloading.
 */
template <class v>
struct Type2Type
{
    typedef v value;
};

}

#endif // UTILS_HPP
