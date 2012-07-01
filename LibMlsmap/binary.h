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
#include "mlsmap.h"
#include "surface.h"
#include "vectorcell.h"
#include "field.h"

/// First two bytes of a field (used for debugging)
#define BINARY_FIELD_START  (('I' << 8 ) | 'F')

/**
 * @brief Represents errors that may occure during binary data parsing.
 */
class BinaryException : public std::exception
{
public:
    /**
     * @brief Create exception
     * @param msg Short error description
     */
    BinaryException( std::string msg ) : msg_( msg ) {}

    virtual ~BinaryException() throw() {}
    virtual const char* what() const throw() { return msg_.c_str(); }

private:
    /// Short error description
    std::string msg_;
};

/**
 * @brief Holds information about a field (binary size etc).
 */
struct FieldHeader
{
    /// Field index. Determines the position of the field
    int16_t index;

    /// Binary size of field (in 16-bit words)
    uint32_t binary_size;
};

/**
 * @brief Represents a map header holding all neccessary information
 *         (e.g. the binary format version, cell siez etc).
 */
struct MapHeader
{
    /// Major version number
    char major_version;

    /// Minor version number
    char minor_version;

    /// Size of one cell in centimeters
    uint16_t cell_size;

    /// Gap size in centimeters
    uint16_t gap_size;

    /// Number of allocated fields
    uint16_t num_fields;
};

/**
 * @brief Provides methods to write and read the binary map format.
 * Only supported map type is currently MLSmap<VectorCell>.
 */
class Binary
{
public:

    /**
     * @brief Generate binary representation of a surface.
     *
     * @param s The surface
     * @param b Append the binary representation to this vector.
     */
    static void writeSurface( const Surface& s, std::vector<int16_t>& b );

    /**
     * @brief Create a surface object from binary data.
     *
     * @param b Vector holding the binary data
     * @param i Start reading at b[i] and increment i. After this call i will
     *        point to the first word that does not belong to the surface binary representation.
     *
     * @throws BinaryException If size of b is too small.
     * @return The surface
     */
    static Surface readSurface( const std::vector<int16_t>& b, std::size_t& i );

    /**
     * @brief Generate binary representation of a vector cell including all surfaces.
     *
     * @param c The vector cell
     * @param cell_index Index of the cell
     * @param b Append the binary representation to this vector.
     */
    static void writeCell( const VectorCell& c, uint16_t cell_index, std::vector<int16_t> &b );

    /**
     * @brief Create a vector cell object from binary data.
     *
     * @param c The vector containing all cells
     * @param b Vector holding the binary data
     * @param i Start reading at b[i] and increment i. After this call i will
     *        point to the first word that does not belong to the vector cell binary representation.
     *
     * @throws BinaryException If size of b is too small to hold the complete vector cell.
     */
    static void readCell( std::vector<VectorCell>& c, const std::vector<int16_t> &b, std::size_t& i );

    /**
     * @brief Generate binary representation of a field including all cells and surfaces.
     *
     * @param f The field
     * @param f_index Index of the field. Used to determine the position of the field.
     * @param b Append the binary representation to this vector.
     */
    static void writeField( const Field<VectorCell>& f,
                           const int16_t f_index,
                           std::vector<int16_t>& b );

    /**
     * @brief Create a field from binary data.
     *
     * @param h Field header holding the binary size etc
     * @param f Field to initialize
     * @param b Vector holding the binary data
     * @param i Start reading at b[i] and increment i. After this call i will
     *        point to the first word that does not belong to the field binary representation.
     *
     * @throws BinaryException If the field header checksum fails of if the data vector is
     *        too small to hold the complete field.
     */
    static void readField( const FieldHeader &h,
                           Field<VectorCell>* f,
                           const std::vector<int16_t>& b,
                           std::size_t& i );

    /**
     * @brief Read binary field header information.
     *
     * @param h Write the data to this parameter
     * @param b Vector holding the binary data
     * @param i Start reading at b[i] and increment i. After this call i will
     *        point to the first word that does not belong to the field binary representation.
     *
     * @throws BinaryException If the field header checksum fails or if there is not enough data
     *        to hold a complete field header.
     */
    static void readFieldHeader( FieldHeader& h,
                                 const std::vector<int16_t>& b,
                                 std::size_t& i );

    /**
     * @brief Get size of field header representation.
     *
     * @return Size of header (unit is 16-bit words)
     */
    static uint32_t getFieldHeaderSize() {
        return 5;
    }

    /**
     * @brief Generate a map header including all neccessary information (e.g. f cell size).
     *
     * @param map The map
     * @param b Append header data to this vector
     */
    static void writeMapHeader( MLSmap<VectorCell>* map,
                                std::vector<int16_t>& b );

    /**
     * @brief Read map header data.
     *
     * @param h Map header to initialize
     * @param b Vector holding the binary data
     * @param i Start reading at b[i] and increment i. After this call i will
     *        point to the first word that does not belong to the header data.
     *
     * @throws BinaryException If the header checksum fails of if the data vector
     *        is too small to hold a complete header.
     */
    static void readMapHeader( MapHeader& header,
                               const std::vector<int16_t> &b,
                               std::size_t& i );

    /**
     * @brief Get size of header representation.
     *
     * @return Size of header (unit is 16-bit words)
     */
    static uint32_t getMapHeaderSize() {
        return 8;
    }

    /**
     * @brief Calculate binary field size in 16-bit words.
     * @return Binary field size in 16-bit words including the header information.
     */
    static uint32_t getFieldSize( const Field<VectorCell>& f );

    /**
     * @brief Calculate a (simple) checksum
     *
     * @param b The data
     * @param i Start at index i
     * @param n Use n entries
     *
     * @return The checksum
     */
    static int16_t checksum( const std::vector<int16_t> &b, std::size_t i, std::size_t n );
};

#endif // BINARY_H
