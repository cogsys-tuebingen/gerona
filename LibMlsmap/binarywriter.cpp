/**
 * @file binarywriter.cpp
 * @date June 2012
 * @author marks
 */

// C/C++
#include <inttypes.h>
#include <vector>

// Project
#include "binary.h"
#include "binarywriter.h"

using namespace std;

BinaryWriter::BinaryWriter( MLSmap<VectorCell>* map )
    : map_( map ),
      filesize_( 0 ),
      verbose_( false )
{
}

bool BinaryWriter::write( std::string filename )
{
    // Try to open the file
    ofstream out( filename.c_str());
    if ( !out.good())
        return false;

    // Write file header
    vector<int16_t> b;
    Binary::writeMapHeader( map_, b );
    filesize_ = b.size();
    writeDataVector( b, out );
    b.clear();

    // Write fields
    const vector<Field<VectorCell>*>& f = map_->getFields();
    size_t size = f.size();
    for ( size_t i = 0; i < size; ++i ) {
        if ( f[i] != NULL ) {
            Binary::writeField( *f[i], (int16_t)i, b );
            writeDataVector( b, out );
            filesize_ += 2*b.size();
            b.clear();

            if ( verbose_ ) {
                cout << "Writing field with index " << i << " and binary size " << Binary::getFieldSize( *f[i] ) << endl;
            }
        }
    }
    return true;
}

void BinaryWriter::writeDataVector( const std::vector<int16_t> &b, std::ofstream &out )
{
    vector<int16_t>::const_iterator it = b.begin();
    int16_t c;
    while ( it != b.end()) {
        c = *it;
        out.write( (char*)&c, 2 );
        ++it;
    }
}
