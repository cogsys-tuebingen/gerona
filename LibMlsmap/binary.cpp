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

void Binary::writeSurface( const Surface &s, std::vector<int16_t> &b )
{
    b.push_back( s.length );
    b.push_back( s.height );
    b.push_back( s.varHeight );
    b.push_back( s.type );
}

Surface Binary::readSurface( const std::vector<int16_t> &b, std::size_t& i )
{
    Surface s( 0, 0, 0 );
    s.length = b[i++];
    s.height = b[i++];
    s.varHeight = b[i++];
    s.type = b[i++];
    return s;
}

void Binary::writeCell( const VectorCell &c, uint16_t cell_index, std::vector<int16_t> &b )
{
    const vector<Surface>& s = c.getSurfaces();

    // Cell index
    b.push_back( cell_index );

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
        writeSurface( s[i], b );
}

void Binary::readCell( std::vector<VectorCell>& c, const std::vector<int16_t> &b, std::size_t& i )
{
    // Read cell index
    uint16_t cell_index = b[i++];

    // Read number of surfaces
    uint16_t size = b[i++];

    // Read all surfaces
    for ( unsigned int j = 0; j < size; ++j )
        c[cell_index].addSurface( readSurface( b, i ));
}

void Binary::writeField( const Field<VectorCell> &f, const int16_t f_index, std::vector<int16_t> &b )
{
    // Size of binary field in 16-bit words
    uint32_t f_size = getFieldSize( f );
    b.reserve( f_size + b.size() + 5 );
    b.push_back( BINARY_FIELD_START );
    b.push_back( f_size & 0xFFFF );
    b.push_back((f_size >> 16) & 0xFFFF );

    // Field index
    b.push_back( f_index );

    // Checksum
    b.push_back( checksum( b, b.size() - 3, 3 ));

    // Write cells
    size_t size = f.cells.size();
    for ( size_t i = 0; i < size; ++i ) {
        if ( f.cells[i].getSurfaces().size() > 0 )
            writeCell( f.cells[i], i, b );
    }
}

void Binary::readField( const FieldHeader& h,
                        Field<VectorCell> *f,
                        const std::vector<int16_t> &b,
                        std::size_t& i )
{
    // Minimum size
    if ( b.size() - i < h.binary_size - getFieldHeaderSize())
        throw BinaryException( "Data vector is too small to hold a complete field" );

    // Read all cells
    size_t size = i + h.binary_size - getFieldHeaderSize();
    while ( i < size )
        readCell( f->cells, b, i );
}

void Binary::readFieldHeader( FieldHeader &h, const std::vector<int16_t> &b, std::size_t& i )
{
    // Size?
    if ( b.size() - i < getFieldHeaderSize())
        throw BinaryException( "Data vector is too small to hold a field header" );

    // Field start?
    if ( b[i++] != BINARY_FIELD_START )
        throw BinaryException( "Start of field data != field start" );

    // Binary field size
    h.binary_size = 0;
    h.binary_size |= (uint32_t)b[i++] & 0x0000FFFF;
    h.binary_size |= ((uint32_t)b[i++] << 16) & 0xFFFF0000;

    // Index
    h.index = b[i++];

    // Checksum?
    if ( checksum( b, i - 3, 3 ) != b[i++] )
        throw BinaryException( "Field header checksum failed" );
}

void Binary::writeMapHeader( MLSmap<VectorCell>* map, std::vector<int16_t> &b )
{
    // File id 'MLSMAP'
    b.push_back( 19533 );
    b.push_back( 19795 );
    b.push_back( 20545 );

    // Major and minor format version (currently always 1.0)
    b.push_back( 1 << 8 );

    // Cell and gap size
    b.push_back((int16_t)map->getCellSize());
    b.push_back((int16_t)map->getGapSize());


    // Number of allocated fields
    b.push_back( map->getFieldNumber());

    // Header checksum
    b.push_back( checksum( b, b.size() - 7, 7 ));
}

void Binary::readMapHeader( MapHeader& h,
                            const std::vector<int16_t> &b,
                            std::size_t &i )
{
    // Data size?
    if ( b.size() - i < getMapHeaderSize())
        throw BinaryException( "Data vector is too small to hold a map header" );

    // Check start bytes (should be MLSMAP)
    if ( b[i++] != 19533 || b[i++] != 19795 || b[i++] != 20545 )
        throw BinaryException( "Start of data != \"MLSMAP\"" );

    // Check version (only 1.0 supported)
    h.major_version = (b[i] >> 8) & 0xFF;
    h.minor_version = b[i++] & 0xFF;

    // Cell and gap size, number of allocated
    h.cell_size = b[i++];
    h.gap_size = b[i++];
    h.num_fields = b[i++];

    // Checksum? You never know...
    if ( checksum( b, i - 7, 7 ) != b[i++] )
        throw BinaryException( "Map header checksum failed" );
}

uint32_t Binary::getFieldSize( const Field<VectorCell> &f )
{
    uint32_t f_size = getFieldHeaderSize(); // Header
    size_t size = f.cells.size();
    for ( size_t i = 0; i < size; ++i ) {
        if ( f.cells[i].getSurfaces().size() > 0 ) {
            f_size += 2; // Cell header
            f_size += 4 * f.cells[i].getSurfaces().size();
        }
    } /// @todo Overflow check
    return f_size;
}

int16_t Binary::checksum( const std::vector<int16_t> &b, std::size_t i, std::size_t n )
{
    /// @todo Use CRC or something...
    int16_t c = 0;
    std::size_t max = i + n;
    for ( ; i < max; ++i )
        c += b[i];
    return c;
}
