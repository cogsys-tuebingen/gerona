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
      filesize_( 0 )
{
}

bool BinaryWriter::write( std::string filename )
{
    // Try to open the file
    ofstream out( filename.c_str());
    if ( !out.good())
        return false;

    // Write file header
    writeHeader( out );
    filesize_ = 10;

    // Write fields
    vector<int16_t> b;
    const vector<Field<VectorCell>*>& f = map_->getFields();
    size_t size = f.size();
    for ( size_t i = 0; i < size; ++i ) {
        if ( f[i] != NULL ) {
            Binary::fromField( *f[i], (int16_t)i, b );
            writeDataVector( b, out );
            filesize_ += 2*b.size();
            b.clear();
        }
    }
    return true;
}

void BinaryWriter::writeHeader( std::ofstream &out )
{
    // File id 'MLSMAP'
    char id[] = {'M', 'L', 'S', 'M', 'A', 'P'};
    out.write( id, sizeof( id ));

    // Major format version
    char c = 1;
    out.write( &c, 1 );

    // Minor format version
    c = 0;
    out.write( &c, 1 );

    /// @todo Number of cells per field etc

    // Number of allocated fields
    uint16_t f_num = map_->getFieldNumber();
    out.write((char*)&f_num, 2 );
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
