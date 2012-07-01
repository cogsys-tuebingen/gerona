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
    : map_( map ), verbose_( false )
{
}

void BinaryReader::read( std::string filename )
{
    // Open file
    ifstream in( filename.c_str());
    if ( !in.good())
        throw BinaryException( "Cannot open file" );

    // Read header
    vector<int16_t> b;
    std::size_t i( 0 );
    readDataVector( b, Binary::getMapHeaderSize(), in );
    MapHeader mh;
    Binary::readMapHeader( mh, b, i );

    // Map header info
    if ( verbose_ ) {
        cout << "Map header:\n"
             << " - Cell size: " << mh.cell_size << "\n"
             << " - Gap size:  " << mh.gap_size << "\n"
             << " - Fields:    " << mh.num_fields << endl;
    }

    // Check file version
    if ( mh.major_version != 1 || mh.minor_version != 0 )
        throw BinaryException( "Unsupported format version" );

    // Header looks good
    map_->setCellSize( mh.cell_size );
    map_->setGapSize( mh.gap_size );

    // Read fields
    FieldHeader fh;
    for ( uint16_t j = 0; j < mh.num_fields; ++j ) {
        // Read field header
        b.clear(); i = 0;
        readDataVector( b, Binary::getFieldHeaderSize(), in );
        Binary::readFieldHeader( fh, b, i );

        // Field info
        if ( verbose_ ) {
            cout << "Field header #" << j << "\n"
                 << " - Index:    " << fh.index << "\n"
                 << " - Bin size: " << fh.binary_size << endl;
        }

        // Check
        if ( fh.binary_size == 0 )
            throw BinaryException( "Field with binary size 0" );

        // Read field data
        b.clear(); i = 0;
        readDataVector( b, fh.binary_size - Binary::getFieldHeaderSize(), in );

        // Parse data
        Field<VectorCell>* f = new Field<VectorCell>;
        Binary::readField( fh, f, b, i );
        map_->setField( f, fh.index );
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
            throw BinaryException( "IO error or unexpected end of file" );
        d = (buff[1] << 8) & 0xFF00;
        d |= buff[0] & 0x00FF;
        b.push_back( d );
    }
}
