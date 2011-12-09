// ConvertUnit.h: Constants for conversions between different units.
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

#ifndef __CONVERTUNIT_H
#define __CONVERTUNIT_H

const double METERS_TO_FEET = 3.280839895013;
const double FEET_TO_METERS = 1 / METERS_TO_FEET;
const double KM_TO_NM = 1.853;
const double NM_TO_KM = 1 / KM_TO_NM;
const double KM_TO_MI = FEET_TO_METERS * 5.28;
const double MI_TO_KM = 1 / KM_TO_MI;
// 1 knoten = 1 seemeile/h = 1852m/h = 1852/3600 m/sec
const double KNOTS_TO_MPS = 1852/3600;	/* Knots to meters per second */

#endif
