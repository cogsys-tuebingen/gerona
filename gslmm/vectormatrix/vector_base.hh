//
// $Id: vector_base.hh,v 1.18 2008-02-13 15:50:59 cholm Exp $ 
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
#ifndef GSLMM_vector_base
#define GSLMM_vector_base

/** @file   vectormatrix/vector_base.hh
    @author Christian Holm
    @date   Mon Sep 16 04:46:14 2002
    @brief  Declaration of vector classses. 
    A base template class as well as some specialisation are declared
    in this file. */

#ifndef __CMATH__
# include <cmath>
#endif
#ifndef __VECTOR__
# include <vector>
#endif
#ifndef __CASSERT__
# include <cassert>
#endif
#ifndef __IOSTREAM__
# include <iostream>
#endif
#include <memory>
#ifndef GSLMM_math_type_trait
# include <gslmm/math/type_trait.hh>
#endif

namespace gslmm
{
  /** @defgroup vector Vectors 
      @ingroup vectormatrix
  */
  /** @class vector vector_base.hh <gslmm/vectormatrix/vector_base.hh>
      @brief Vector class (base template).

      This class is the uninstanised version of the vector class. All
      other vector classes have a similar interface.  The instantised
      specialised vector classes are: 
      <ul> 
        <li> gslmm::vector<long double,long double>       </li>
        <li> gslmm::vector<double,double>                 </li>
        <li> gslmm::vector<float,float>                   </li>
        <li> gslmm::vector<unsigned long,unsigned long>   </li>
        <li> gslmm::vector<long,long>                     </li>
        <li> gslmm::vector<unsigned int,unsigned int>     </li>
        <li> gslmm::vector<int,int>                       </li>
        <li> gslmm::vector<unsigned short,unsigned short> </li>
        <li> gslmm::vector<short,short>                   </li>
        <li> gslmm::vector<unsigned char,unsigned char>   </li>
        <li> gslmm::vector<char,char>                     </li>
      </ul>  
      The base class is of little use, at as it will not really do
      anything. In other words, one <strong>must</strong> specialise the
      `derived' classes.  

      @ingroup vector
      @todo Make I/O member functions. 
      @todo make outer product member function.
      @todo Perhaps the view copy constructor could be private?. 
      @todo What about the assignment operator in the face of non-RVO
  */
  template<typename T>
  class vector;
  template<typename T>
  class distribution;

  //__________________________________________________________________
  template <typename T> 
  struct vector_return
  {
    /** @param v Do the operation, and store in-place the result in
	@a v */
    virtual void operator()(vector<T>& v) const = 0;
    /** @return The size of the vector resulting from operation */
    virtual size_t size() const = 0;
      /** Destructor */
    virtual ~vector_return() {}
  };
  
  //==================================================================
  template <typename T> 
  struct vector_oper
  {
    typedef vector<T>                  vector_type;
    typedef vector_return<T>           return_type;
    typedef std::auto_ptr<return_type> ptr_type;
  };

  //================================================================
  /** @brief Trait of matrix-matrix operation 
   */
  template <typename T, template <class> class Arg>
  struct v_arg_trait;

  //________________________________________________________________
  /** @brief Trait of matrix-matrix operation, specialised for
      matrix<T> 
  */
  template <typename T>
  struct v_arg_trait<T, vector>
  {
    /** Type of matrix */
    typedef vector<T> vector_type;
    /** Type of argument to ctor */
    typedef const vector<T>& ctor_type;
    /** Stored type */
    typedef const vector<T>& member_type;
    /** type in @c operator* */
    typedef const vector<T>& oper_type;
    /** type in @c doit argument */
    typedef const vector<T>& doit_type;
    /** Assert that @a m is valid */
    static void check(ctor_type ) {}
    /** Get the row size of @a m */
    static size_t size(member_type m) { return m.size(); }
    /** Apply and destroy */
    static void apply(member_type m, vector_type& c) { c = m; }
    enum { temp = 0  };
  };


  //________________________________________________________________
  /** @brief Trait of vector operation, specialised for vector_return<T> 
   */
  template <typename T>
  struct v_arg_trait<T, vector_return>
  {
    /** Type of matrix */
    typedef vector<T> vector_type;
    /** Type of argument to ctor */
    typedef typename vector_oper<T>::ptr_type ctor_type;
    /** Stored type */
    typedef typename vector_oper<T>::ptr_type member_type;
    /** type in @c operator* */
    typedef typename vector_oper<T>::ptr_type oper_type;
    /** type in @c doit argument */
    typedef const typename vector_oper<T>::ptr_type& doit_type;
    /** Assert that @a m is valid */
    static void check(doit_type m) { assert(m.get()); }
    /** Get the row size of @a m */
    static size_t size(doit_type m) { return m->size(); }
    /** Apply and destroy */
    static void apply(doit_type m, vector_type& c) { m->operator()(c); }
    enum { temp = 1 };
  };
  
  //__________________________________________________________________
  /** @{ */
  /** @name General operations on vectors. */
  /** Add two vectors and return new vector.
      @ingroup vector
      @param w first vector. 
      @param u second.
      @return the vector @f$ v@f$ where @f$ v_i = w_i + u_i @f$ */
  template <typename T>
  inline vector<T> 
  operator+(const vector<T>& w, const vector<T>& u) 
  { 
    vector<T> v(w, true); v += u; return v; 
  }
  //__________________________________________________________________
  /** Subtract two vectors and return new vector. 
      @ingroup vector
      @param w first vector. 
      @param u second.
      @return the vector @f$ v@f$ where @f$ v_i = w_i - u_i @f$  */
  template <typename T>
  inline vector<T> 
  operator-(const vector<T>& w, const vector<T>& u) 
  { 
    vector<T> v(w, true); v -= u; return v; 
  }
  //__________________________________________________________________
  /** Form dot product between two vectors
      @ingroup vector
      @param w vector to multiply with. 
      @param u vector to multiply with. 
      @return the dot product @f$ r = \sum_i w_i u_i @f$ */
  template <typename T>
  inline T
  operator*(const vector<T>& w, const vector<T>& u) 
  {
    T ret;
    for (size_t i = 0; i < w.size(); i++) ret += w[i] * u[i];
    return ret;
  }
#if 0
  //__________________________________________________________________
  /** Divide two vectors element by element and return the result.
      @ingroup vector
      @param w numerator vector.
      @param u denomniator vector.
      @return the vector @f$ v@f$ where @f$ v_i = w_i / u_i @f$  */
  template <typename T>
  inline vector<T> 
  operator/(const vector<T>& w, const vector<T>& u) 
  {
    vector<T> v(w, true); v /= u; return v; 
  }
#endif
  /** @} */

  //__________________________________________________________________
  /** @{ */
  /** @name General operations on vector-scalers. */
  /** Add a number to a vector and return new vector.
      @ingroup vector
      @param w the vector to add to. 
      @param u the number to add.
      @return the vector @f$ v@f$ where @f$ v_i = w_i + u @f$ */
  template <typename T>
  inline vector<T> 
  operator+(const vector<T>& w, const T& u) 
  { 
    vector<T> v(w, true); v += u; return v; 
  }
  //__________________________________________________________________
  /** Subtract a number from a vector and return new vector.       
      @ingroup vector
      @param w the vector to subtract from. 
      @param u number.
      @return the vector @f$ v@f$ where @f$ v_i = w_i - u @f$ */
  template <typename T>
  inline vector<T> 
  operator-(const vector<T>& w, const T& u) 
  { 
    vector<T> v(w, true); v -= u; return v; 
  }
  //__________________________________________________________________
  /** Multiply a vector by a number and return new vector.  
      @ingroup vector
      @param w vector to mulitply. 
      @param u number to multiply with. 
      @return the vector @f$ v@f$ where @f$ v_i = w_i \cdot u @f$  */
  template <typename T>
  inline vector<T> 
  operator*(const vector<T>& w, const T& u) 
  {
    vector<T> v(w, true); v *= u; return v; 
  }
  //__________________________________________________________________
  /** Divide a vector by a number and return the result.
      @ingroup vector
      @param w numerator vector.
      @param u the number to divide by.
      @return the vector @f$ v@f$ where @f$ v_i = w_i / u @f$  */
  template <typename T>
  inline vector<T> 
  operator/(const vector<T>& w, const T& u) 
  {
    vector<T> v(w, true); v /= u; return v; 
  }
  /** @} */
  //____________________________________________________________________  
  /** @{ */
  /** @name Output streamers on vectors */
  /** Output a vector to stream (formated)
      @ingroup vector
      @param o The output stream
      @param v The vector. 
      @return the output stream. */
  template <typename T>
  std::ostream& operator<<(std::ostream& o, const vector<T>& v) 
  {
    for (size_t i = 0; i < v.size(); i++) 
      o << (i == 0 ? "[" : ",") << v[i];
    return o << "]";
  }
  /** @} */
}

#endif
//____________________________________________________________________
//
// EOF
//
