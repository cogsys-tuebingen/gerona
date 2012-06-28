/**
 * @file binary.cpp
 * @date June 2012
 * @author marks
 */

// C/C++
#include <limits>
#include <iostream>

// Project
#include "binary.h"

using namespace std;

void Binary::fromSurface( const Surface &s, std::vector<int16_t> &b )
{
    b.push_back( s.length );
    b.push_back( s.height );
    b.push_back( s.varHeight );
    b.push_back( s.type );
}

Surface Binary::toSurface( const std::vector<int16_t> &b, std::size_t& i )
{
    Surface s( 0, 0, 0 );
    s.length = b[i++];
    s.height = b[i++];
    s.varHeight = b[i++];
    s.type = b[i++];
    return s;
}

void Binary::fromCell( const VectorCell &c, std::vector<int16_t> &b )
{
    const vector<Surface>& s = c.getSurfaces();

    // Write number of surfaces (max 65535)
    uint16_t size;
    if ( s.size() <= numeric_limits<uint16_t>::max()) {
        size = (uint16_t)s.size();
    } else {
        cout << "WARN: Vector cell with more than "
             << numeric_limits<uint16_t>::max()
             << " surfaces. Skipping surfaces!" << endl;
        size = numeric_limits<uint16_t>::max();
    }
    b.push_back( size );

    // Write all surfaces
    for ( unsigned int i = 0; i < size; ++i )
        fromSurface( s[i], b );
}

void Binary::toCell( VectorCell& c, const std::vector<int16_t> &b, std::size_t& i )
{
    // Read number of surfaces
    uint16_t size = b[i++];

    // Read all surfaces
    for ( unsigned int j = 0; j < size; ++j ) {
        c.addSurface( toSurface( b, i ));
    }
}

void Binary::fromField( const Field<VectorCell> &f, const int16_t f_index, std::vector<int16_t> &b )
{
    // Size of binary field in 16-bit words
    uint32_t f_size = getFieldSize( f );
    b.reserve( f_size + b.size());

    // Start byte and size
    b.push_back( BINARY_FIELD_START );
    b.push_back( (f_size >> 16) & 0xFFFF );
    b.push_back( f_size & 0xFFFF );
    cout << "Writing field with size: " << f_size << " and index: " << f_index << endl;

    // Field index
    b.push_back( f_index );

    // Write cells
    size_t size = f.cells.size();
    for ( size_t i = 0; i < size; ++i ) {
        fromCell( f.cells[i], b );
    }
}

bool Binary::toField( int16_t& f_index,
                      Field<VectorCell> *f,
                      const std::vector<int16_t> &b,
                      std::size_t i )
{
    // Minimum size
    if ( b.size() - i < 4 )
        return false;

    // Start bytes?
    if ( b[i++] != (int16_t)BINARY_FIELD_START )
        return false;

    // Size?
    uint32_t f_size = 0;
    f_size |= ((uint32_t)b[i++] << 16) & 0xFFFF0000;
    f_size |= (uint32_t)b[i++] & 0x0000FFFF;
    if ( f_size - 4 > b.size() - i )
        return false;

    // Field index
    f_index = b[i++];

    // Read all cells
    size_t size = f->cells.size();
    for ( size_t j = 0; j < size; ++j ) {
        toCell( f->cells[j], b, i );
    }

    return true;
}

uint32_t Binary::getFieldSize( const Field<VectorCell> &f )
{
    uint32_t f_size = 4; // Header
    size_t size = f.cells.size();
    for ( size_t i = 0; i < size; ++i ) {
        f_size += 1; // Number of surfaces
        f_size += 4 * f.cells[i].getSurfaces().size();
    } /// @todo Overflow check
    return f_size;
}

