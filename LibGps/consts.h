// Consts.h: Constants and enums used in GPS Thing.
//
// Copyright (C) 1998 by Jason Bevins
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License (COPYING.txt) for more details.
//

#ifndef __CONSTS_H
#define __CONSTS_H

#include <string>
using namespace std;
// These are types of coordinates that exist within the application.
enum COORD_TYPE
{
	COORD_LATLON = 0,
	COORD_UTM    = 1
};

// These are the types of coordinate display modes for lat/lon that exist
// within the application.
enum COORD_LATLON_FORMAT
{
	FORMAT_D   = 0,
	FORMAT_DM  = 1,
	FORMAT_DMS = 2
};

// These are the types of units that GPS Thing supports.
enum UNIT_TYPE
{
	UNIT_NAUTICAL = 0,
	UNIT_STATUTE  = 1,
	UNIT_METRIC   = 2
};

// The two axis types: x (longitude or easting), and y (latitude or nothing.)
enum AXIS_TYPE
{
	AXIS_X = 0,
	AXIS_Y = 1
};

// Hemisphere direction; horizontal (N/S), vertical (E/W).
enum HEMISPHERE_DIRECTION
{
	HEMISPHERE_NS = 0,
	HEMISPHERE_EW = 1
};

// Validity of the GPS connection.
enum GPS_STATUS
{
	STATUS_OK = 0,              // Connection to GPS is valid.
	STATUS_SERIAL_TIMEOUT = 1,  // Connection timed out.
	STATUS_NO_CONNECTION = 2,   // GPS is not connected.
};

// Serial port parity.
enum SETPARITY {
	SETPARITY_NONE = 0,
	SETPARITY_ODD = 1,
	SETPARITY_EVEN = 2
};

// Number of stop bits in the data.
enum STOPBITS {
	STOPBITS_ONE = 0,
	STOPBITS_ONEFIVE = 1,
	STOPBITS_TWO = 2
};

// GPS coordinate fix quality.
enum GPS_FIX_QUALITY
{
	FIX_INVALID = 0,  // Invalid fix (poor coverage)
	FIX_GPS_OK = 3,   // Valid fix
	FIX_DGPS_OK = 5,  // Valid fix with DGPS corrections.
};

// This enumerated type defines an object size within GPS Thing's map view.
enum SIZE_TYPE
{
	SIZE_SMALL = 0,
	SIZE_MEDIUM = 1,
	SIZE_LARGE = 2
};

// This enumerated type is used as the return type for several functions that
// handle exceptions.
enum EXCEPTION_TYPE
{
	EXCEPTION_OK = 0,              // No error occurred.
	EXCEPTION_OUT_OF_MEMORY = 1,   // Required memory could not be allocated.
	EXCEPTION_OUT_OF_RESOURCES = 2 // Required resources could not be created.
};


// The four types of calibration data
const int DATA_BMP_X = 0;
const int DATA_BMP_Y = 1;
const int DATA_MAP_X = 2;
const int DATA_MAP_Y = 3;

// Length of time before serial port times out, in milliseconds.
const int COMM_GPS_TIMEOUT = 1750;

// Defaults for GPS Thing.
const uint8 DEFAULT_COM_PORT = 2;
const uint32 DEFAULT_BAUD_RATE = 4800;
const uint8 DEFAULT_DATABITS = 8;
const SETPARITY DEFAULT_PARITY = SETPARITY_NONE;
const STOPBITS DEFAULT_STOPBITS = STOPBITS_ONE;
const COORD_LATLON_FORMAT DEFAULT_LATLON_FORMAT = FORMAT_DMS;
const COORD_TYPE DEFAULT_COORD_TYPE = COORD_LATLON;
const UNIT_TYPE DEFAULT_UNIT_TYPE = UNIT_NAUTICAL;
// Colors are in 0x00BBGGRR format.
const SIZE_TYPE DEFAULT_OBJECT_SIZE = SIZE_SMALL;

// Mmmmm, pi...
const double PI = 3.141592653589793;

// Radians to degrees conversion constants.
const double RAD_TO_DEG = 180.0 / PI;
const double DEG_TO_RAD = PI / 180.0;

// The registry key containing the application settings for GPS Thing.
const string REG_SETTINGS_KEY = "Settings";
const string REG_TOOLBAR_KEY = "Toolbar";

// This user-defined value will be returned from the GetProfileXxx()
// functions if there is an error reading a value from the Registry.
const int REG_ERROR = 0xFFFFFF80;


#endif
