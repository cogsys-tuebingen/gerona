// SatData.h: Definition of the CSatData class.
//
// Copyright (C) 1998, 1999 by Jason Bevins
//
// This class holds information about a particular GPS satellite.
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

#ifndef __SATDATA_H
#define __SATDATA_H

#include "mytypes.h"


struct CSatData
{
	CSatData ()
	{
		prn       = 0;
		elevation = 0;
		azimuth   = 0;
		strength  = 0;
	}

	CSatData (uint8 prn_, uint16 elevation_, uint16 azimuth_,
		uint16 strength_)
	{
		prn       = prn_;
		elevation = elevation_;
		azimuth   = azimuth_;
		strength  = strength_;
	}

	uint16 prn;       // Satellite's ID.
	uint16 elevation; // Elevation of satellite, in degrees.
	uint16 azimuth;   // Azimuth of satellite, in degrees.
	uint16 strength;  // Signal strength of satellite.
};

#endif
