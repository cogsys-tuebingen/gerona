/**
 * @file binary.h
 * @date June 2012
 * @author marks
 */

#ifndef BINARY_H
#define BINARY_H

// C/C++
#include <vector>
#include <inttypes.h>

// Project
#include "surface.h"
#include "vectorcell.h"
#include "field.h"

/// First two bytes of a field (very simpel data consistency check)
#define BINARY_FIELD_START  1122

/**
 * @brief Provides methods to write and read the binary map format.
 */
class Binary
{
public:

    static void fromSurface( const Surface& s, std::vector<int16_t>& b );

    static Surface toSurface( const std::vector<int16_t>& b, std::size_t& i );

    static void fromCell( const VectorCell& c, std::vector<int16_t>& b );

    static void toCell( VectorCell& c, const std::vector<int16_t> &b, std::size_t& i );

    static void fromField( const Field<VectorCell>& f,
                           const int16_t f_index,
                           std::vector<int16_t>& b );

    static bool toField( int16_t& f_index,
                         Field<VectorCell>* f,
                         const std::vector<int16_t>& b,
                         std::size_t i );

    /**
     * @brief Calculate binary field size in 16-bit words.
     * @return Binary field size in 16-bit words including the header information.
     */
    static uint32_t getFieldSize( const Field<VectorCell>& f );
};

#endif // BINARY_H
