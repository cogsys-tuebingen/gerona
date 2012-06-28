/**
 * @file binarywriter.h
 * @date June 2012
 * @author marks
 */

#ifndef BINARYWRITER_H
#define BINARYWRITER_H

// C/C++
#include <string>
#include <fstream>

// Project
#include "vectorcell.h"
#include "mlsmap.h"

class BinaryWriter
{
public:
    BinaryWriter( MLSmap<VectorCell>* map );

    bool write( std::string filename );

    /**
     * @brief Get size of written file (last one)
     * @return Size of written file in bytes
     */
    std::size_t getFileSize() const { return filesize_; }

private:

    void writeHeader( std::ofstream& out );

    void writeDataVector( const std::vector<int16_t>& b, std::ofstream& out );

    /// The map
    MLSmap<VectorCell>* map_;

    /// Filesize of written file (in bytes)
    std::size_t filesize_;
};

#endif // BINARYWRITER_H
