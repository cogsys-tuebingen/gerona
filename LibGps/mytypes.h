// MyTypes.h: Miscellaneous portable variable types.
//
// This header file defines the data types used by these classes.  You'll
// need to change this file if your compiler defines different lengths for
// these types.
//
// If bool is defined by your compiler (it's now apparantly in the ANSI
// standard for C++), you'll need to comment out the lines in this file
// containing 'bool'.
//
// This file was originally used by other GPS projects I was working on (the
// NMEA parser class and the NMEA serial port reader class.)
//
// Copyright (C) 1998, 1999 by Jason Bevins
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

#ifndef __MYTYPES_H
#define __MYTYPES_H

//typedef int bool;
//const bool true  = 1;
//const bool false = 0;

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;
typedef char           int8;
typedef short          int16;
typedef long           int32;
typedef unsigned int   uint;

#endif
