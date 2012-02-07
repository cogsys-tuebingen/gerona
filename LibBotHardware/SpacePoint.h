/*
 * @file  SpacePoint.h
 *
 * @date  early 21st century
 * @author karsten bohlmann bohlmann@gmail.com
 *
 */



#ifndef __SPACEPOINT_H__
#define __SPACEPOINT_H__


#include <string>
#include <vector>
// stdbool.h necessary for hid.h in c++
#include <stdbool.h>
#include <hid.h>
#include "Global.h"

using namespace std;

const unsigned short SPACEPOINT_VID = 0x20ff;
const unsigned short SPACEPOINT_PID = 0x0100;

// spacepoint fusion provides two usb endpoints:
// raw data and filtered "game data"
const unsigned int SPACEPOINT_ENDPOINT_RAW = 0x81;
const unsigned int SPACEPOINT_ENDPOINT_FILTERED = 0x82;
const unsigned int SPACEPOINT_LENGTH_RAW = 20;
const unsigned int SPACEPOINT_LENGTH_FILTERED = 15;

/**
 * @class SpacePoint
 *
 * @brief This class provides functions for the 3D orientation sensor
 * Spacepoint Fusion by PNIcorp.com
 * @author Karsten Bohlmann
 */
class SpacePoint
{
public:
    /// Constructs an instance of SpacePoint.
    SpacePoint();
    /// Destroys the instance of SpacePoint.
    /** The scanning process is stopped, if running. */
    ~SpacePoint();

    /**
         Establishes a connection
         @return true if successful
         */
    bool Connect();
    /// Closes the connection through the usb port
    void Close();

    bool Init ();

    /**
      @param[out] accels scaled acceleration values
      @param[out] quat orientation as quaternion
      */
    bool ReadFilteredData(FVector& accels, FVector& quat);
    bool ReadRawData (IVector& gyrosXYZ, IVector& mags);
private:




    HIDInterfaceMatcher mHidMatcher;
    HIDInterface        *mHid;
    bool                mIsConnected;

};

#endif
