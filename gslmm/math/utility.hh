//
// $Id: utility.hh,v 1.5 2006-05-01 14:25:23 cholm Exp $ 
//  
//  gslmm::utility
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
#ifndef GSL_utility
#define GSL_utility
/** @file   utility.hh
    @author Christian Holm
    @date   Fri Feb 28 22:19:07 2003
    @brief  Various mathematical utility functions. */
#ifndef __GSL_MATH_H__
#include  <gsl/gsl_math.h>
#endif

/** @defgroup utility Mathematical utility functions. */
namespace gslmm
{
  //@{
  /// @name Test functions
  //___________________________________________________________________
  /** Test if a number is undefined. 
      @ingroup utility
      @param x the number to test
      @return true if number is not a number, false otherwise. */
  template <typename Type>
  inline bool is_not_a_number(const Type& x) 
  {
    return gsl_isnan(double(x)) == 0 ? false : true;
  }
  //___________________________________________________________________
  /** Test if a number is +/- infinity.
      @ingroup utility
      @param x the number to test
      @return +1 if @f$ x = +\infty@f$, -1 if @f$ x = -\infty@f$, 
      or 0. */
  template <typename Type>
  inline int is_infinite(const Type& x)  
  {
    return gsl_isinf(double(x));
  }
  //___________________________________________________________________
  /** Test if a number is finite.
      @ingroup utility
      @param x number to test.
      @return  This function returns true if @a x is a real number,
      and false if it is infinite or not-a-number. */
  template <typename Type>
  inline bool is_finite(const Type& x)  
  {
    return gsl_finite(x) == 1 ? true : false;
  }
  //___________________________________________________________________
  /** Test if a number is odd. 
      @ingroup utility
      @param x number to test. 
      @return true if number (as an short) is odd, false
      otherwise. */ 
  template <typename Type>
  inline bool is_odd(const Type& x) 
  { 
    return GSL_IS_ODD(short(x)); 
  }
  //___________________________________________________________________
  /** Test if a number is even. 
      @ingroup utility
      @param x number to test. 
      @return true if number isn't odd, false otherwise. */
  template <typename Type>
  inline bool is_even(const Type& x) 
  { 
    return GSL_IS_EVEN(short(x)); 
  }
  //___________________________________________________________________
  /** Get sign of number
      @ingroup utility
      @param x number to test. 
      @return -1 if negative, +1 otherwise. */
  template <typename Type>
  inline int sign(const Type& x) 
  { 
    return GSL_SIGN(x); 
  }
  //@}
  //@{
  /// @name Comparisons
  //___________________________________________________________________
  /** Return maximum of arguments 
      @ingroup utility
      @param a first to compare 
      @param b second to compare 
      @return if @f$ a>b@f$ return @a a, otherwise @a b */
  template <typename Type>
  inline Type max(const Type& a, const Type& b)  
  {
    return GSL_MAX(a,b);
  }
  //___________________________________________________________________
  /** Return minimum of arguments 
      @ingroup utility
      @param a first to compare 
      @param b second to compare 
      @return if @f$ a<b@f$ return @a a, otherwise @a b */
  template <typename Type>
  inline Type min(const Type& a, const Type& b)  
  {
    return GSL_MIN(a,b);
  }
  //___________________________________________________________________
  /** Compare to floating point numbers up to some realtive accuracy. 
      The relative accuracy is measured using an interval of size 
      @f$ 2\delta@f$, where @f$ \delta = 2^k \epsilon@f$ and @f$ k@f$
      is the maximimum base-2 exponent of @a x and @a y as computed by
      the function frexp(). 

      If @a x and @a y lie within this interval, they are considered 
      approximately equal and the function returns 0. Otherwise if
      @f$ x < y@f$, the function returns -1, or if @f$ x > y@f$, the
      function returns +1.

      Note that the arguments are promoted to double before the
      comparison. 

      @ingroup utility
      @param x is the first floating point number to compare. 
      @param y is the second floating point number to compare. 
      @param e is the desired relative accuracy.  A good candidate is
      one of the members of gslmm::type_trait<Type>. 
      @return 0 if @f$ x = y @f$ within the accuracy, else +1 if if @f$ x >
      y@f$, or -1 if @f$ x < y@f$. 
  */
  template <typename Type> 
  inline int compare(const Type& x, const Type& y, const Type& e) 
  {
    return gsl_fcmp(static_cast<const double>(x), 
		    static_cast<const double>(y),
		    static_cast<const double>(e));
  }
  //@}

  //===================================================================
  //@{
  ///@name Miscellaneous functions
  /** Compute @f$ \log(1+x)@f$ in a way that is accurate for small 
      @a x. */
  template <typename Type>
  inline Type log1p(const Type& x) 
  {
    return Type(gsl_log1p(static_cast<const double>(x)));
  }  
  //___________________________________________________________________
  /** Compute @f$ \exp(x)-1@f$ in a way that is accurate for small 
      @a x. */
  template <typename Type>
  inline Type expm1(const Type& x) 
  {
    return Type(gsl_expm1(static_cast<const double>(x)));
  }
  //___________________________________________________________________
  /** Compute @f$ \sqrt{x^2+y^2}@f$ */
  template <typename Type>
  inline Type hypot(const Type& x, const Type& y) 
  {
    return Type(gsl_hypot(static_cast<const double>(x),
			  static_cast<const double>(y)));
  }
  //___________________________________________________________________
  /** Compute @f$ \cosh^{-1}{x}@f$ */
  template <typename Type>
  inline Type acosh(const Type& x)
  {
    return Type(gsl_acosh(static_cast<const double>(x)));
  }
  //___________________________________________________________________
  /** Compute @f$ \sinh^{-1}{x}@f$ */
  template <typename Type>
  inline Type asinh(const Type& x)
  {
    return Type(gsl_asinh(static_cast<const double>(x)));
  }
  //___________________________________________________________________
  /** Compute @f$ \tanh^{-1}{x}@f$ */
  template <typename Type>
  inline Type atanh(const Type& x)
  {
    return Type(gsl_atanh(static_cast<const double>(x)));
  }
  //___________________________________________________________________
  /** Divide two numbers  @f$ x/y@f$ 
      @ingroup utility
  */
  template <typename Type> 
  inline Type divide(const Type& x, const Type& y) 
  {
    return Type(gsl_fdiv(static_cast<const double>(x),
			 static_cast<const double>(y)));
  }
  //@}

  //@{
  /// @name Fraction/Exponent representation.
  //___________________________________________________________________
  /** Compute @f$ x 2^e@f$ */
  template <typename Type>
  inline Type ldexp(const Type x, int e)
  {
    return Type(gsl_ldexp(static_cast<const double>(x),e));
  }
  //___________________________________________________________________
  /** Split number into fraction and exponent.  
      This function splits the number @f$ x@f$ into its normalized
      fraction @f$ f@f$ and exponent @f$ e @f$, such that @f$ x = f
      2^e @f$ and @f$ 0.5 <= f < 1. @f$.
      @ingroup utility
      @param x The number to split. 
      @param e the exponent 
      @return the fractional part @f$ f @f$. */
  template <typename Type>
  inline Type frexp(const Type x, int& e)
  {
    return Type(gsl_frexp(static_cast<const double>(x),&e));
  }
  //@}

  //@{
  ///@name Integer powers 
  //___________________________________________________________________
  /** Calculate the 2-power of a number. 
      @ingroup utility
  */
  template <typename Type> 
  inline Type pow_2(const Type& x) 
  {
    return x*x;
  }
  //___________________________________________________________________
  /** Calculate the 3-power of a number. 
      @ingroup utility
  */
  template <typename Type> 
  inline Type pow_3(const Type& x) 
  {
    return x*x*x;
  }
  //___________________________________________________________________
  /** Calculate the 4-power of a number. 
      @ingroup utility
  */
  template <typename Type> 
  inline Type pow_4(const Type& x) 
  {
    Type x2 = x*x;
    return x2*x2;
  }
  //___________________________________________________________________
  /** Calculate the 5-power of a number. 
      @ingroup utility
  */
  template <typename Type> 
  inline Type pow_5(const Type& x) 
  {
    Type x2 = x*x;
    return x2*x2*x;
  }
  //___________________________________________________________________
  /** Calculate the 6-power of a number. 
      @ingroup utility
  */
  template <typename Type> 
  inline Type pow_6(const Type& x) 
  {
    Type x2 = x*x;
    return x2*x2*x2;
  }
  //___________________________________________________________________
  /** Calculate the 7-power of a number. 
      @ingroup utility
  */
  template <typename Type> 
  inline Type pow_7(const Type& x) 
  {
    Type x3 = x*x*x;
    return x3*x3*x;
  }
  //___________________________________________________________________
  /** Calculate the 8-power of a number. 
      @ingroup utility
  */
  template <typename Type> 
  inline Type pow_8(const Type& x) 
  {
    Type x2 = x*x;
    Type x4 = x2*x2;
    return x4*x4;
  }
  //___________________________________________________________________
  /** Calculate the 9-power of a number. 
      @ingroup utility
  */
  template <typename Type> 
  inline Type pow_9(const Type& x) 
  {
    Type x3 = x*x*x;
    return x3*x3*x3;
  }
  //___________________________________________________________________
  /** Calculate the n-power of a number. 
      @ingroup utility
  */
  template <typename Type> 
  inline Type pow_int(const Type& x, int n) 
  {
    return Type(gsl_pow_int(static_cast<const double>(x), n));
  }
  //@}
}

#endif
//____________________________________________________________________
//
// EOF
//
