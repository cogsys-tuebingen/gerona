//
// $Id: constant.hh,v 1.9 2006-04-26 10:06:04 cholm Exp $ 
//  
//  gslmm::constant
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
#ifndef GSL_constant
#define GSL_constant
#ifndef __GSL_MATH_H__
#include <gsl/gsl_math.h>
#endif
/** @file   math/constant.hh
    @author Christian Holm
    @date   Wed Mar 12 15:24:19 2003
    @brief  Mathematical constants */


namespace gslmm
{
  /** @class constant math/constant.hh <gslmm/math/constant.hh>
      @brief Mathematical contants 
      @ingroup constant 
  */
  struct constant
  {
    /** @return The base of exponentials, @f$ e@f$ */
    static const double e();
    /** @return The base-2 logarithm of e, @f$\log_2(e)@f$ */
    static const double log2_of_e();
    /** @return The base-10 logarithm of e, @f$\log_10(e)@f$ */
    static const double log10_of_e();
    /** @return The square root of two, @f$\sqrt 2@f$ */
    static const double sqrt_of_2();
    /** @return The square root of one-half, @f$\sqrt{\frac12}@f$ */
    static const double sqrt_of_one_half();
    /** @return The square root of three, @f$\sqrt 3@f$ */
    static const double sqrt_of_3();
    /** @return The static constant @f$\pi@f$, @f$\pi@f$ */
    static const double pi();
    /** @return @f$\pi@f$ divided by two, @f$\pi/2@f$ */
    static const double half_of_pi();
    /** @return @f$\pi@f$ divided by four, @f$\pi/4@f$ */
    static const double quarter_of_pi();
    /** @return The square root of @f$\pi@f$, @f$\sqrt\pi@f$ */
    static const double sqrt_of_pi();
    /** @return Two divided by the square root of @f$\pi@f$, @f$2/\sqrt\pi@f$ */
    static const double two_over_sqrt_of_pi();
    /** @return The reciprocal of @f$\pi@f$, @f$\frac1\pi@f$ */
    static const double one_over_pi();
    /** @return Twice the reciprocal of @f$\pi@f$, @f$\frac2\pi@f$ */
    static const double two_over_pi();
    /** @return The natural logarithm of ten, @f$\log(10)@f$ */
    static const double log_of_10();
    /** @return The natural logarithm of two, @f$\log(2)@f$ */
    static const double log_of_2();
    /** @return The natural logarithm of @f$\pi@f$, @f$\log(\pi)@f$ */
    static const double log_of_pi();
    /** @return Euler's static constant, @f$\gamma@f$ */
    static const double euler();
    /** @return Positive infinity, @f$ +1/0@f$ */
    static const double minus_infinity();
    /** @return Negative infinity, @f$ -1/0@f$ */
    static const double plus_infinity();
    /** @return Not a number (undefined), @f$ 0/0@f$ */
    static const double not_a_number();
  };

  inline const double constant::e()                   { return M_E; }
  inline const double constant::log2_of_e()           { return M_LOG2E; }
  inline const double constant::log10_of_e()          { return M_LOG10E; }
  inline const double constant::sqrt_of_2()           { return M_SQRT2; }
  inline const double constant::sqrt_of_one_half()    { return M_SQRT1_2; }
  inline const double constant::sqrt_of_3()           { return M_SQRT3; }
  inline const double constant::pi()                  { return M_PI; }
  inline const double constant::half_of_pi()          { return M_PI_2; }
  inline const double constant::quarter_of_pi()       { return M_PI_4; }
  inline const double constant::sqrt_of_pi()          { return M_SQRTPI; }
  inline const double constant::two_over_sqrt_of_pi() { return M_2_SQRTPI; }
  inline const double constant::one_over_pi()         { return M_1_PI; }
  inline const double constant::two_over_pi()         { return M_2_PI; }
  inline const double constant::log_of_10()           { return M_LN10; }
  inline const double constant::log_of_2()            { return M_LN2; }
  inline const double constant::log_of_pi()           { return M_LNPI; }
  inline const double constant::euler()               { return M_EULER; }
  inline const double constant::plus_infinity()       { return gsl_posinf(); }
  inline const double constant::minus_infinity()      { return gsl_neginf(); }
  inline const double constant::not_a_number()        { return gsl_nan(); }
  // inline const double constant::plus_infinity()       { return double(GSL_POSINF); }
  //  inline const double constant::minus_infinity()      { return double(GSL_NEGINF); }
  // inline const double constant::not_a_number()        { return double(GSL_NAN); }
}

#endif
//____________________________________________________________________
//
// EOF
//
