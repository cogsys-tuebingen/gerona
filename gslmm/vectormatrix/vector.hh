//
// $Id: vector.hh,v 1.5 2006-05-01 14:23:45 cholm Exp $ 
//  
//  gslmm::vector
//  Copyright (C) 2002 Christian Holm Christensen <cholm@nbi.dk> 
//
//  This library is free software; you can redistribute it and/or 
//  modify it under the terms of the GNU Lesser General Public License 
//  as published by the Free Software Foundation; either version 2.1 
//  of the License, or (at your option) any later version. 
//
//  This library is distributed in the hope that it will be useful, 
//  but WITHOUT ANY WARRANTY; without even the implied warranty of 
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
//  Lesser General Public License for more details. 
// 
//  You should have received a copy of the GNU Lesser General Public 
//  License along with this library; if not, write to the Free 
//  Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 
//  02111-1307 USA 
//
#ifndef GSLMM_vector_vector
#define GSLMM_vector_vector

/** @file   vector.hh
    @author Christian Holm
    @date   Mon Sep 16 04:46:14 2002
    @brief  Declaration of vector classses. 
    A base template class as well as some specialisation are declared
    in this file. */

/** @defgroup vectormatrix Vectors and maticies 
    @todo Use the BLAS functions to implement matrix-vector and
    matrix-matrix multiplication, as well as possibly some other
    stuff. 
 */

#ifndef GSLMM_vector_double
#include <gslmm/vectormatrix/vector_double.hh>
#endif
#ifndef GSLMM_vector_long_double
#include <gslmm/vectormatrix/vector_long_double.hh>
#endif
#ifndef GSLMM_vector_float
#include <gslmm/vectormatrix/vector_float.hh>
#endif
#ifndef GSLMM_vector_unsigned_long
#include <gslmm/vectormatrix/vector_unsigned_long.hh>
#endif
#ifndef GSLMM_vector_long
#include <gslmm/vectormatrix/vector_long.hh>
#endif
#ifndef GSLMM_vector_unsigned_int
#include <gslmm/vectormatrix/vector_unsigned_int.hh>
#endif
#ifndef GSLMM_vector_int
#include <gslmm/vectormatrix/vector_int.hh>
#endif
#ifndef GSLMM_vector_unsigned_short
#include <gslmm/vectormatrix/vector_unsigned_short.hh>
#endif
#ifndef GSLMM_vector_short
#include <gslmm/vectormatrix/vector_short.hh>
#endif
#ifndef GSLMM_vector_unsigned_char
#include <gslmm/vectormatrix/vector_unsigned_char.hh>
#endif
#ifndef GSLMM_vector_char
#include <gslmm/vectormatrix/vector_char.hh>
#endif

#endif
//____________________________________________________________________
//
// EOF
//
