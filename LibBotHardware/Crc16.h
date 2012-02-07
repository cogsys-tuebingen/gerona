/** @file Crc16.h
  * @brief This file contains functions for the crc16 checksum.
  **/

#include <string>
#include <bitset>
#include <sstream>
#include <vector>

/** Converts the <EM> sz </EM> first bytes of <EM> bytes </EM> into a bit-string.
  * If <EM> sz </EM> is zero, then all bytes of <EM> bytes </EM> are taken.
  **/

std::string bytesToBits(std::vector<unsigned char> bytes, unsigned sz=0)
{
    unsigned size = sz ? sz : bytes.size();

    std::ostringstream ost;
    for (unsigned i=0; i!=size; ++i) {
        ost << std::bitset<8>(bytes[i]);
    }

    return ost.str();
}

/** Returns the rest of the modulo operation. If zero, the CRC16-test was passed. **/
unsigned calculateCRC16(std::string datastream)
{
  unsigned poly = 0x1021; // CRC-16 polynomial
  unsigned crc16 = 0; // shift register

  for (unsigned i=0; i<datastream.size(); ++i) {
    if (((crc16 & 0x8000) ? 1 : 0) != (datastream[i] != '0')) {
      crc16 = (crc16<<1) ^ poly;
    }
    else {
      crc16 <<= 1;
    }
  }

  return crc16 & 0xFFFF;
}

/** Returns true if datastream passes the CRC16-test; otherwise returns false. **/
bool checkCRC16(std::string datastream)
{
  return !calculateCRC16(datastream);
}
