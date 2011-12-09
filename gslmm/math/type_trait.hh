//
// $Id: type_trait.hh,v 1.1 2006-05-01 14:25:23 cholm Exp $ 
//  
//  gslmm::type_trait
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
#ifndef GSL_type_trait
#define GSL_type_trait
#ifndef __GSL_MACHINE_H__
# include  <gsl/gsl_machine.h>
#endif
#ifndef __GSL_PRECISION_H__
# include <gsl/gsl_precision.h>
#endif
#ifndef __LIMITS__
# include <limits>
#endif
/** @file   math/type_trait.hh
    @author Christian Holm
    @date   Wed Mar 12 14:56:27 2003
    @brief  Type constants */


namespace gslmm
{
  /** @class type_trait type_trait.hh <gslmm/math/type_trait.hh>
      @brief Type constants - basic template 
      @ingroup math
  */
  template <typename Type> 
  struct type_trait 
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<Type> numeric_limits;
    /** Type of reference */
    typedef Type& reference_type;
    /** Type of pointer */
    typedef Type* pointer_type;
    /** Type of pointer */
    typedef Type elementary_type;
    
    /** Smallest difference representable by a char variable,
	@return @f$\epsilon_{\mbox{T}}@f$.  */
    static const Type epsilon()          { return numeric_limits::epsilon(); }
    /** Least number representable by a char,
	@return @f$\mbox{min}_{\mbox{T}}@f$.  */
    static const Type min()              { return numeric_limits::min(); }
    /** Largest number representable by a char,
	@return @f$\mbox{max}_{\mbox{T}}@f$.  */
    static const Type max()              { return numeric_limits::max(); }
    /** Return the @e conjugate (if applicable) of @a x)
	@param x Number to conjugate 
	@return @a x */
    static const reference_type conjugate(const Type& x) { return x; }
  };
  
  
  //===================================================================
  /** @brief Type constants for char.  
      @ingroup math 
  */
  template <> 
  struct type_trait<char>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<char> numeric_limits;
    /** Type of reference */
    typedef char& reference_type;
    /** Type of pointer */
    typedef char* pointer_type;
    /** elementary type */
    typedef char elementary_type;
    /** multicative neutral element */  
    static const char unit()             { return 1; }
    /** additive neutral element */  
    static const char null()             { return '\0'; }
    /** Smallest difference representable by a char variable,
	@return @f$\epsilon_{\mbox{char}}@f$.  */
    static const char epsilon()          { return numeric_limits::epsilon(); }
    /** Least number representable by a char,
	@return @f$\mbox{min}_{\mbox{char}}@f$.  */
    static const char min()              { return numeric_limits::min(); }
    /** Largest number representable by a char,
	@return @f$\mbox{max}_{\mbox{char}}@f$.  */
    static const char max()              { return numeric_limits::max(); }
    /** @param x Number to "conjugate"
	@return @a x */
    static const char conjugate(const char& x) { return x; }
  };

  //===================================================================
  /** @brief Type constants for unsigned char.  
      @ingroup math 
  */
  template <> 
  struct type_trait<unsigned char>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<unsigned char> numeric_limits;
    /** Type of reference */
    typedef unsigned char& reference_type;
    /** Type of pointer */
    typedef unsigned char* pointer_type;
    /** elementary type */
    typedef unsigned char elementary_type;
    /** multicative neutral element */  
    static const unsigned char unit() { return 1U; }
    /** additive neutral element */  
    static const unsigned char null() { return 0U; }
    /** Smallest difference representable by a unsigned char variable,
	@return @f$\epsilon_{\mbox{unsigned char}}@f$.  */
    static const unsigned char epsilon() { return numeric_limits::epsilon(); }
    /** Least number representable by a unsigned char,
	@return @f$\mbox{min}_{\mbox{unsigned char}}@f$.  */
    static const unsigned char min()  { return numeric_limits::min(); }
    /** Largest number representable by a unsigned char,
	@return @f$\mbox{max}_{\mbox{unsigned char}}@f$.  */
    static const unsigned char max()  { return numeric_limits::max(); }
    /** @param x Number to "conjugate"
	@return @a x */
    static const unsigned char conjugate(const unsigned char& x) { return x; }
  };

  //===================================================================
  /** @brief Type constants for short.  
      @ingroup math 
  */
  template <> 
  struct type_trait<short>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<short> numeric_limits;
    /** Type of reference */
    typedef short& reference_type;
    /** Type of pointer */
    typedef short* pointer_type;
    /** elementary type */
    typedef short elementary_type;
    /** multicative neutral element */  
    static const short unit()             { return 1; }
    /** additive neutral element */  
    static const short null()             { return 0; }
    /** Smallest difference representable by a short variable,
	@return @f$\epsilon_{\mbox{short}}@f$.  */
    static const short epsilon()          { return numeric_limits::epsilon(); }
    /** Least number representable by a short,
	@return @f$\mbox{min}_{\mbox{short}}@f$.  */
    static const short min()              { return numeric_limits::min(); }
    /** Largest number representable by a short,
	@return @f$\mbox{max}_{\mbox{short}}@f$.  */
    static const short max()              { return numeric_limits::max(); }
    /** @param x Number to "conjugate"
	@return @a x */
    static const short conjugate(const short& x) { return x; }
  };

  //===================================================================
  /** @brief Type constants for unsigned short.  
      @ingroup math 
  */
  template <> 
  struct type_trait<unsigned short>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<unsigned short> numeric_limits;
    /** Type of reference */
    typedef unsigned short& reference_type;
    /** Type of pointer */
    typedef unsigned short* pointer_type;
    /** elementary type */
    typedef unsigned short elementary_type;
    /** multicative neutral element */  
    static const unsigned short unit() { return 1U; }
    /** additive neutral element */  
    static const unsigned short null() { return 0U; }
    /** Smallest difference representable by a unsigned short variable,
	@return @f$\epsilon_{\mbox{unsigned short}}@f$.  */
    static const unsigned short epsilon() { return numeric_limits::epsilon(); }
    /** Least number representable by a unsigned short,
	@return @f$\mbox{min}_{\mbox{unsigned short}}@f$.  */
    static const unsigned short min()  { return numeric_limits::min(); }
    /** Largest number representable by a unsigned short,
	@return @f$\mbox{max}_{\mbox{unsigned short}}@f$.  */
    static const unsigned short max()  { return numeric_limits::max(); }
    /** @param x Number to "conjugate"
	@return @a x */
    static const unsigned short conjugate(const unsigned short& x) { return x;}
  };

  //===================================================================
  /** @brief Type constants for int.  
      @ingroup math 
  */
  template <> 
  struct type_trait<int>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<int> numeric_limits;
    /** Type of reference */
    typedef int& reference_type;
    /** Type of pointer */
    typedef int* pointer_type;
    /** elementary type */
    typedef int elementary_type;
    /** multicative neutral element */  
    static const int unit()             { return 1; }
    /** additive neutral element */  
    static const int null()             { return 0; }
    /** Smallest difference representable by a int variable,
	@return @f$\epsilon_{\mbox{int}}@f$.  */
    static const int epsilon()          { return numeric_limits::epsilon(); }
    /** Least number representable by a int,
	@return @f$\mbox{min}_{\mbox{int}}@f$.  */
    static const int min()              { return numeric_limits::min(); }
    /** Largest number representable by a int,
	@return @f$\mbox{max}_{\mbox{int}}@f$.  */
    static const int max()              { return numeric_limits::max(); }
    /** @param x Number to "conjugate"
	@return @a x */
    static const int conjugate(const int& x) { return x; }
  };
  
  //===================================================================
  /** @brief Type constants for unsigned int.  
      @ingroup math 
  */
  template <> 
  struct type_trait<unsigned int>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<unsigned int> numeric_limits;
    /** Type of reference */
    typedef unsigned int& reference_type;
    /** Type of pointer */
    typedef unsigned int* pointer_type;
    /** elementary type */
    typedef unsigned int elementary_type;
    /** multicative neutral element */  
    static const unsigned int unit() { return 1U; }
    /** additive neutral element */  
    static const unsigned int null() { return 0U; }
    /** Smallest difference representable by a unsigned int variable,
	@return @f$\epsilon_{\mbox{unsigned int}}@f$.  */
    static const unsigned int epsilon() { return numeric_limits::epsilon(); }
    /** Least number representable by a unsigned int,
	@return @f$\mbox{min}_{\mbox{unsigned int}}@f$.  */
    static const unsigned int min()  { return numeric_limits::min(); }
    /** Largest number representable by a unsigned int,
	@return @f$\mbox{max}_{\mbox{unsigned int}}@f$.  */
    static const unsigned int max()  { return numeric_limits::max(); }
    /** @param x Number to "conjugate"
	@return @a x */
    static const unsigned int conjugate(const unsigned int& x) { return x; }
  };

  //===================================================================
  /** @brief Type constants for long.  
      @ingroup math 
  */
  template <> 
  struct type_trait<long>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<long> numeric_limits;
    /** Type of reference */
    typedef long& reference_type;
    /** Type of pointer */
    typedef long* pointer_type;
    /** elementary type */
    typedef long elementary_type;
    /** multicative neutral element */  
    static const long unit()             { return 1L; }
    /** additive neutral element */  
    static const long null()             { return 0L; }
    /** Smallest difference representable by a long variable,
	@return @f$\epsilon_{\mbox{long}}@f$.  */
    static const long epsilon()          { return numeric_limits::epsilon(); }
    /** Least number representable by a long,
	@return @f$\mbox{min}_{\mbox{long}}@f$.  */
    static const long min()              { return numeric_limits::min(); }
    /** Largest number representable by a long,
	@return @f$\mbox{max}_{\mbox{long}}@f$.  */
    static const long max()              { return numeric_limits::max(); }
    /** @param x Number to "conjugate"
	@return @a x */
    static const long conjugate(const long& x) { return x; }
  };

  //===================================================================
  /** @brief Type constants for unsigned long.  
      @ingroup math 
  */
  template <> 
  struct type_trait<unsigned long>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<unsigned long> numeric_limits;
    /** Type of reference */
    typedef unsigned long& reference_type;
    /** Type of pointer */
    typedef unsigned long* pointer_type;
    /** elementary type */
    typedef unsigned long elementary_type;
    /** multicative neutral element */  
    static const unsigned long unit() { return 1UL; }
    /** additive neutral element */  
    static const unsigned long null() { return 0UL; }
    /** Smallest difference representable by a unsigned long variable,
	@return @f$\epsilon_{\mbox{unsigned long}}@f$.  */
    static const unsigned long epsilon() { return numeric_limits::epsilon(); }
    /** Least number representable by a unsigned long,
	@return @f$\mbox{min}_{\mbox{unsigned long}}@f$.  */
    static const unsigned long min()  { return numeric_limits::min(); }
    /** Largest number representable by a unsigned long,
	@return @f$\mbox{max}_{\mbox{unsigned long}}@f$.  */
    static const unsigned long max()  { return numeric_limits::max(); }
    /** @param x Number to "conjugate"
	@return @a x */
    static const unsigned long conjugate(const unsigned long& x) { return x; }
  };

  //===================================================================
  /** @brief Type constants for @c float.  
      @ingroup math
  */
  template <>
  struct type_trait<float> 
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<float> numeric_limits;
    /** Type of reference */
    typedef float& reference_type;
    /** Type of pointer */
    typedef float* pointer_type;
    /** elementary type */
    typedef float elementary_type;
    /** multicative neutral element */  
    static const float unit()             { return 1.f; }
    /** additive neutral element */  
    static const float null()             { return 0.f; }

    /** Smallest difference representable by a float variable,
	@return @f$\epsilon_{\mbox{float}}@f$.  */
    static const float epsilon()          { return GSL_FLT_EPSILON; }
    /** @return @f$\sqrt{\epsilon_{\mbox{float}}}@f$. */
    static const float sqrt_of_epsilon()  { return GSL_SQRT_FLT_EPSILON; }
    /** @return @f$\sqrt[3]{\epsilon_{\mbox{float}}}@f$. */
    static const float root3_of_epsilon() { return GSL_ROOT3_FLT_EPSILON; }
    /** @return @f$\sqrt[4]{\epsilon_{\mbox{float}}}@f$. */
    static const float root4_of_epsilon() { return GSL_ROOT4_FLT_EPSILON; }
    /** @return @f$\sqrt[5]{\epsilon_{\mbox{float}}}@f$. */
    static const float root5_of_epsilon() { return GSL_ROOT5_FLT_EPSILON; }
    /** @return @f$\sqrt[6]{\epsilon_{\mbox{float}}}@f$. */
    static const float root6_of_epsilon() { return GSL_ROOT6_FLT_EPSILON; }
    /** @return @f$\log{\epsilon_{\mbox{float}}}@f$. */
    static const float log_of_epsilon()   { return GSL_LOG_FLT_EPSILON; }

    /** Least number representable by a float,
	@return @f$\mbox{min}_{\mbox{float}}@f$.  */
    static const float min()              { return GSL_FLT_MIN; } 
    /** @return @f$\sqrt{\mbox{min}_{\mbox{float}}}@f$. */
    static const float sqrt_of_min()      { return GSL_SQRT_FLT_MIN; }
    /** @return @f$\sqrt[3]{\mbox{min}_{\mbox{float}}}@f$. */
    static const float root3_of_min()     { return GSL_ROOT3_FLT_MIN; }
    /** @return @f$\sqrt[4]{\mbox{min}_{\mbox{float}}}@f$. */
    static const float root4_of_min()     { return GSL_ROOT4_FLT_MIN; }
    /** @return @f$\sqrt[5]{\mbox{min}_{\mbox{float}}}@f$. */
    static const float root5_of_min()     { return GSL_ROOT5_FLT_MIN; }
    /** @return @f$\sqrt[6]{\mbox{min}_{\mbox{float}}}@f$. */
    static const float root6_of_min()     { return GSL_ROOT6_FLT_MIN; }
    /** @return @f$\log{\mbox{min}_{\mbox{float}}}@f$. */
    static const float log_of_min()       { return GSL_LOG_FLT_MIN; }

    /** Largest number representable by a float,
	@return @f$\mbox{max}_{\mbox{float}}@f$.  */
    static const float max()              { return GSL_FLT_MAX; }
    /** @return @f$\sqrt{\mbox{max}_{\mbox{float}}}@f$. */
    static const float sqrt_of_max()      { return GSL_SQRT_FLT_MAX; }
    /** @return @f$\sqrt[3]{\mbox{max}_{\mbox{float}}}@f$. */
    static const float root3_of_max()     { return GSL_ROOT3_FLT_MAX; }
    /** @return @f$\sqrt[4]{\mbox{max}_{\mbox{float}}}@f$. */
    static const float root4_of_max()     { return GSL_ROOT4_FLT_MAX; }
    /** @return @f$\sqrt[5]{\mbox{max}_{\mbox{float}}}@f$. */
    static const float root5_of_max()     { return GSL_ROOT5_FLT_MAX; }
    /** @return @f$\sqrt[6]{\mbox{max}_{\mbox{float}}}@f$. */
    static const float root6_of_max()     { return GSL_ROOT6_FLT_MAX; }
    /** @return @f$\log{\mbox{max}_{\mbox{float}}}@f$. */
    static const float log_of_max()       { return GSL_LOG_FLT_MAX; }

    /** @param x Number to "conjugate"
	@return @a x */
    static const float conjugate(const float& x) { return x; }
  };


  //===================================================================
  /** @brief Type constants for double.  
      @ingroup math 
  */
  template <> 
  struct type_trait<double>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<double> numeric_limits;
    /** Type of reference */
    typedef double& reference_type;
    /** Type of pointer */
    typedef double* pointer_type;
    /** elementary type */
    typedef double elementary_type;
    /** multicative neutral element */  
    static const double unit()             { return 1.; }
    /** additive neutral element */  
    static const double null()             { return 0.; }
     
    /** Smallest difference representable by a double variable,
	@return @f$\epsilon_{\mbox{double}}@f$.  */
    static const double epsilon()          { return GSL_DBL_EPSILON; }
    /** @return @f$\sqrt{\epsilon_{\mbox{double}}}@f$. */
    static const double sqrt_of_epsilon()  { return GSL_SQRT_DBL_EPSILON; }
    /** @return @f$\sqrt[3]{\epsilon_{\mbox{double}}}@f$. */
    static const double root3_of_epsilon() { return GSL_ROOT3_DBL_EPSILON; }
    /** @return @f$\sqrt[4]{\epsilon_{\mbox{double}}}@f$. */
    static const double root4_of_epsilon() { return GSL_ROOT4_DBL_EPSILON; }
    /** @return @f$\sqrt[5]{\epsilon_{\mbox{double}}}@f$. */
    static const double root5_of_epsilon() { return GSL_ROOT5_DBL_EPSILON; }
    /** @return @f$\sqrt[6]{\epsilon_{\mbox{double}}}@f$. */
    static const double root6_of_epsilon() { return GSL_ROOT6_DBL_EPSILON; }
    /** @return @f$\log{\epsilon_{\mbox{double}}}@f$. */
    static const double log_of_epsilon()   { return GSL_LOG_DBL_EPSILON; }

    /** Least number representable by a double,
	@return @f$\mbox{min}_{\mbox{double}}@f$.  */
    static const double min()              { return GSL_DBL_MIN; }
    /** @return @f$\sqrt{\mbox{min}_{\mbox{double}}}@f$. */
    static const double sqrt_of_min()      { return GSL_SQRT_DBL_MIN; }
    /** @return @f$\sqrt[3]{\mbox{min}_{\mbox{double}}}@f$. */
    static const double root3_of_min()     { return GSL_ROOT3_DBL_MIN; }
    /** @return @f$\sqrt[4]{\mbox{min}_{\mbox{double}}}@f$. */
    static const double root4_of_min()     { return GSL_ROOT4_DBL_MIN; }
    /** @return @f$\sqrt[5]{\mbox{min}_{\mbox{double}}}@f$. */
    static const double root5_of_min()     { return GSL_ROOT5_DBL_MIN; }
    /** @return @f$\sqrt[6]{\mbox{min}_{\mbox{double}}}@f$. */
    static const double root6_of_min()     { return GSL_ROOT6_DBL_MIN; }
    /** @return @f$\log{\mbox{min}_{\mbox{double}}}@f$. */
    static const double log_of_min()       { return GSL_LOG_DBL_MIN; }

    /** Largest number representable by a double,
	@return @f$\mbox{max}_{\mbox{double}}@f$.  */
    static const double max()              { return GSL_DBL_MAX; }
    /** @return @f$\sqrt{\mbox{max}_{\mbox{double}}}@f$. */
    static const double sqrt_of_max()      { return GSL_SQRT_DBL_MAX; }
    /** @return @f$\sqrt[3]{\mbox{max}_{\mbox{double}}}@f$. */
    static const double root3_of_max()     { return GSL_ROOT3_DBL_MAX; }
    /** @return @f$\sqrt[4]{\mbox{max}_{\mbox{double}}}@f$. */
    static const double root4_of_max()     { return GSL_ROOT4_DBL_MAX; }
    /** @return @f$\sqrt[5]{\mbox{max}_{\mbox{double}}}@f$. */
    static const double root5_of_max()     { return GSL_ROOT5_DBL_MAX; }
    /** @return @f$\sqrt[6]{\mbox{max}_{\mbox{double}}}@f$. */
    static const double root6_of_max()     { return GSL_ROOT6_DBL_MAX; }
    /** @return @f$\log{\mbox{max}_{\mbox{double}}}@f$. */
    static const double log_of_max()       { return GSL_LOG_DBL_MAX; }

    /** @param x Number to "conjugate"
	@return @a x */
    static const double conjugate(const double& x) { return x; }
  };

  //===================================================================
  /** @brief Type constants for long double.  
      @ingroup math 
  */
  template <> 
  struct type_trait<long double>
  {
    /** Standard numerical limits */
    typedef std::numeric_limits<long double> numeric_limits;
    /** Type of reference */
    typedef long double& reference_type;
    /** Type of pointer */
    typedef long double* pointer_type;
    /** elementary type */
    typedef long double elementary_type;
    /** multicative neutral element */  
    static const long double unit()             { return 1.; }
    /** additive neutral element */  
    static const long double null()             { return 0.; }
     
    /** Smallest difference representable by a long double variable,
	@return @f$\epsilon_{\mbox{long double}}@f$.  */
    static const long double epsilon()          { return GSL_DBL_EPSILON; }
    /** @return @f$\sqrt{\epsilon_{\mbox{long double}}}@f$. */
    static const long double sqrt_of_epsilon()  { return GSL_SQRT_DBL_EPSILON;}
    /** @return @f$\sqrt[3]{\epsilon_{\mbox{long double}}}@f$. */
    static const long double root3_of_epsilon() {return GSL_ROOT3_DBL_EPSILON;}
    /** @return @f$\sqrt[4]{\epsilon_{\mbox{long double}}}@f$. */
    static const long double root4_of_epsilon() {return GSL_ROOT4_DBL_EPSILON;}
    /** @return @f$\sqrt[5]{\epsilon_{\mbox{long double}}}@f$. */
    static const long double root5_of_epsilon() {return GSL_ROOT5_DBL_EPSILON;}
    /** @return @f$\sqrt[6]{\epsilon_{\mbox{long double}}}@f$. */
    static const long double root6_of_epsilon() {return GSL_ROOT6_DBL_EPSILON;}
    /** @return @f$\log{\epsilon_{\mbox{long double}}}@f$. */
    static const long double log_of_epsilon()   { return GSL_LOG_DBL_EPSILON; }

    /** Least number representable by a long double,
	@return @f$\mbox{min}_{\mbox{long double}}@f$.  */
    static const long double min()              { return GSL_DBL_MIN; }
    /** @return @f$\sqrt{\mbox{min}_{\mbox{long double}}}@f$. */
    static const long double sqrt_of_min()      { return GSL_SQRT_DBL_MIN; }
    /** @return @f$\sqrt[3]{\mbox{min}_{\mbox{long double}}}@f$. */
    static const long double root3_of_min()     { return GSL_ROOT3_DBL_MIN; }
    /** @return @f$\sqrt[4]{\mbox{min}_{\mbox{long double}}}@f$. */
    static const long double root4_of_min()     { return GSL_ROOT4_DBL_MIN; }
    /** @return @f$\sqrt[5]{\mbox{min}_{\mbox{long double}}}@f$. */
    static const long double root5_of_min()     { return GSL_ROOT5_DBL_MIN; }
    /** @return @f$\sqrt[6]{\mbox{min}_{\mbox{long double}}}@f$. */
    static const long double root6_of_min()     { return GSL_ROOT6_DBL_MIN; }
    /** @return @f$\log{\mbox{min}_{\mbox{long double}}}@f$. */
    static const long double log_of_min()       { return GSL_LOG_DBL_MIN; }

    /** Largest number representable by a long double,
	@return @f$\mbox{max}_{\mbox{long double}}@f$.  */
    static const long double max()              { return GSL_DBL_MAX; }
    /** @return @f$\sqrt{\mbox{max}_{\mbox{long double}}}@f$. */
    static const long double sqrt_of_max()      { return GSL_SQRT_DBL_MAX; }
    /** @return @f$\sqrt[3]{\mbox{max}_{\mbox{long double}}}@f$. */
    static const long double root3_of_max()     { return GSL_ROOT3_DBL_MAX; }
    /** @return @f$\sqrt[4]{\mbox{max}_{\mbox{long double}}}@f$. */
    static const long double root4_of_max()     { return GSL_ROOT4_DBL_MAX; }
    /** @return @f$\sqrt[5]{\mbox{max}_{\mbox{long double}}}@f$. */
    static const long double root5_of_max()     { return GSL_ROOT5_DBL_MAX; }
    /** @return @f$\sqrt[6]{\mbox{max}_{\mbox{long double}}}@f$. */
    static const long double root6_of_max()     { return GSL_ROOT6_DBL_MAX; }
    /** @return @f$\log{\mbox{max}_{\mbox{long double}}}@f$. */
    static const long double log_of_max()       { return GSL_LOG_DBL_MAX; }

    /** @param x Number to "conjugate"
	@return @a x */
    static const long double conjugate(const long double& x) { return x; }
  };


#if 0
  //===================================================================
  /** Type constants for short float.  
      @ingroup math 
  */
  template <>
  struct type_trait<short float> 
  {
    /// Smallest difference representable by a float variable,
    /** @return @f$\epsilon_{\mbox{float}}@f$.  */
    static const float epsilon()          { return GSL_SFLT_EPSILON; }
    /** @return @f$\sqrt{\epsilon_{\mbox{float}}}@f$. */
    static const float sqrt_of_epsilon()  { return GSL_SQRT_SFLT_EPSILON; }
    /** @return @f$\sqrt[3]{\epsilon_{\mbox{float}}}@f$. */
    static const float root3_of_epsilon() { return GSL_ROOT3_SFLT_EPSILON; }
    /** @return @f$\sqrt[4]{\epsilon_{\mbox{float}}}@f$. */
    static const float root4_of_epsilon() { return GSL_ROOT4_SFLT_EPSILON; }
    /** @return @f$\sqrt[5]{\epsilon_{\mbox{float}}}@f$. */
    static const float root5_of_epsilon() { return GSL_ROOT5_SFLT_EPSILON; }
    /** @return @f$\sqrt[6]{\epsilon_{\mbox{float}}}@f$. */
    static const float root6_of_epsilon() { return GSL_ROOT6_SFLT_EPSILON; }
    /** @return @f$\log{\epsilon_{\mbox{float}}}@f$. */
    static const float log_of_epsilon()   { return GSL_LOG_SFLT_EPSILON; }  
  };
#endif

}
#endif
//____________________________________________________________________
//
// EOF
//
