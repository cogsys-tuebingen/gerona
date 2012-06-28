/**
 * @file binaryreader.cpp
 * @date June 2012
 * @author marks
 */

// C/C++
#include <iostream>

// Project
#include "binary.h"
#include "binaryreader.h"

using namespace std;

BinaryReader::BinaryReader( MLSmap<VectorCell> *map )
    : map_( map )
{
}

void BinaryReader::read( std::string filename )
{
    // Open file
    ifstream in( filename.c_str());
    if ( !in.good())
        throw BinaryReaderException( "Cannot open file" );

    vector<int16_t> b;
    readDataVector( b, 5, in );

    // Check start bytes (should be MLSMAP)
    if ( b[0] != 19533 || b[1] != 19795 || b[2] != 20545 )
        throw BinaryReaderException( "Start of file != \"MLSMAP\"" );

    // Check version (only 1.0 supported)
    char minor_ver = b[3] >> 8;
    char major_ver = b[3] & 0xFF;
    if ( major_ver != 1 || minor_ver != 0 )
        throw BinaryReaderException( "File format version not supported" );

    // Read number of fields
    uint16_t num_fields = b[4];
    fields_ = num_fields;

    // Read fields
    for ( uint16_t j = 0; j < num_fields; ++j ) {
        // Check for field header and get field size
        b.clear();
        readDataVector( b, 3, in );

        // Start of field?
        cout << "Field header is: " << b[0] << endl;
        if ( b[0] != BINARY_FIELD_START )
            throw BinaryReaderException( "Invalid field header" );

        // Binary size of field
        uint32_t f_size = 0;
        f_size |= ((uint32_t)b[1] << 16) & 0xFFFF0000;
        f_size |= (uint32_t)b[2] & 0x0000FFFF;
        cout << "Field size is: " << f_size << " (" << b[1] << ", " << b[2] << ")" << endl;

        // Read field data
        readDataVector( b, f_size - 3, in );

        // Parse data
        int16_t f_index = 0;
        size_t i = 0;
        Field<VectorCell>* f = new Field<VectorCell>;
        Binary::toField( f_index, f, b, i );
        map_->setField( f, f_index );

        cout << "Added field with index " << f_index << " successfully" << endl;
    }
}

void BinaryReader::readDataVector( std::vector<int16_t> &b, std::size_t n, std::ifstream &in )
{
    /// @todo optimize this
    char buff[2];
    int16_t d;
    b.reserve( n + b.size());
    for ( size_t i = 0; i < n; ++i ) {
        in.read( buff, 2 );
        if ( i != n - 1 && !in.good())
            throw BinaryReaderException( "IO error or unexpected end of file" );
        d = (buff[1] << 8) & 0xFF00;
        d |= buff[0] & 0x00FF;
        b.push_back( d );
    }
}
