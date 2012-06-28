/**
 * @file binaryreader.h
 * @date June 2012
 * @author marks
 */

#ifndef BINARYREADER_H
#define BINARYREADER_H

// C/C++
#include <string>
#include <fstream>
#include <vector>

// Project
#include "vectorcell.h"
#include "mlsmap.h"

class BinaryReaderException : public std::exception
{
public:
    BinaryReaderException( std::string msg ) : msg_( msg ) {}
    virtual ~BinaryReaderException() throw() {}
    virtual const char* what() const throw() { return msg_.c_str(); }

private:
    std::string msg_;
};

class BinaryReader
{
public:
    BinaryReader( MLSmap<VectorCell>* map );

    void read( std::string filename );

private:

    void readDataVector( std::vector<int16_t>& b, std::size_t n, std::ifstream& in );

    /// The map
    MLSmap<VectorCell>* map_;

    /// Number of allocated fields
    std::size_t fields_;

    /// Number of surfaces
    std::size_t surfaces_;
};

#endif // BINARYREADER_H
