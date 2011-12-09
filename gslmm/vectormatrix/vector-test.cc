//
// $Id: vector-test.cc,v 1.11 2006-05-01 14:23:45 cholm Exp $ 
//  
//  gslmm::error
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
#ifndef __IOSTREAM__
#include <iostream>
#endif
#ifndef GSLMM_math_type_trait
#include <gslmm/math/type_trait.hh>
#endif
#ifndef GSLMM_vector
#include <gslmm/vectormatrix/vector.hh>
#endif
#ifndef GSLMM_test_suite
#include <gslmm/test/test_suite.hh>
#endif
#ifndef __FSTREAM__
#include <fstream>
#endif
#ifndef __IOMANIP__
#include <iomanip>
#endif
#define ts gslmm::test_suite::instance()

/** @file   vector-test.cc
    @author Christian Holm
    @date   Mon Sep 16 13:03:50 2002
    @brief  Test of vector classses.  */

using namespace gslmm;

//____________________________________________________________________  
/// Testing basic allocation and accesses. 
template <typename T> 
void test_basic(typename gslmm::vector<T>::size_type n) 
{
  gslmm::vector<T>*          v_ptr   = new gslmm::vector<T>(n, true);
  gslmm::vector<T>&          v       = *v_ptr;
  bool                       status  = true;
  bool                       status1 = true;
  typename gslmm::vector<T>::iterator i;

  //
  ts.check(v_ptr->data() != 0,   "ctor returns valid pointer");
  ts.check(v_ptr->size() == n,   "ctor returns valid size");
  ts.check(v_ptr->stride() == 1, "ctor returns valid stride");
  
  for (i = v.begin(); i != v.end(); i++) 
    v[i] = T(i);
  
  // 
  for (i = v.begin(); i != v.end(); i++) {
    if (v.data()[i] != T(i)) status = false;
    if (v[i] != T(i))        status1 = false;
  }
  ts.check(status, "operator[] sets the array correctly");
  ts.check(status1, "operator[] gets the elements correctly");

  v.stride(2); 

  // 
  status = true; 
  for (i = v.begin(); i < v.size() / 2; i++) {
    if (v[i] != T(2 * i)) status = false;
    v[i] = T(i + 1000);
  }
  ts.check(status, "operator[] gets correctly with stride"); 

  // 
  status = true;
  for (i = v.begin(); i < v.size() / 2; i++) {
    if (v.data()[2 * i] != T(1000 + i)) status = false;
    v[i] = T(i);
  }
  ts.check(status, "operator[] sets correctly with stride"); 

  v.stride(1);
  for (i = v.begin(); i < v.size(); i++) v[i] = T(i);
  
  // 
  v.swap(2,5);
  status =  (v[2] == 5 && v[5] == 2);

  v.swap(2,5);
  status =  (status && v[2] == 2 && v[5] == 5);  
  ts.check(status, "swap exchanges elemets correctly"); 

  // 
  v.reverse(); 
  status = true;
  for (i = v.begin(); i < v.size() / 2; i++) 
    if (v[i] != T(v.size() - i - 1)) status = false;
  ts.check(status, "reverse ok");
}

//____________________________________________________________________  
/// Testing min/max 
template <typename T>
void test_minmax(typename gslmm::vector<T>::size_type n) 
{
  gslmm::vector<T> v(n);
  for (size_t i = 0; i < v.size(); i++) v[i] = T(i);

  T      exp_max  = v[0];
  T      exp_min  = v[0];
  size_t exp_imax = 0;
  size_t exp_imin = 0;

  for (size_t i = 0; i < v.size(); i++) {
    if (v[i] < exp_min) {
      exp_min  = v[i];
      exp_imin = i;
    }
    if (v[i] > exp_max) {
      exp_max  = v[i];
      exp_imax = i;
    }
  }
  T max       = v.max();
  T min       = v.min();
  size_t imax = v.max_index();
  size_t imin = v.min_index();
  ts.test(max, exp_max, "gslmm::vector::max returns maximum");
  ts.test(imax, exp_imax, "gslmm::vector::max returns index to maximum"); 
  ts.test(min, exp_min, "gslmm::vector::min returns minimum");
  ts.test(imin, exp_imin, "gslmm::vector::min returns index to minimum");

  v.minmax(min, max);
  ts.test(min, exp_min, "gslmm::vector::minmax returns minimum");
  ts.test(max, exp_max, "gslmm::vector::minmax returns maximum");

  v.minmax_index(imin, imax);
  ts.test(imin, exp_imin, "gslmm::vector::minmax returns index to minimum");
  ts.test(imax, exp_imax, "gslmm::vector::minmax returns index to maximum");
}


//____________________________________________________________________  
/// Testing common operations
template <typename T>
void test_operations(typename gslmm::vector<T>::size_type n) 
{
  bool status = true;
  gslmm::vector<T> v(n);    
  gslmm::vector<T> a(n);
  gslmm::vector<T> b(n);
  for (size_t i = 0; i < v.size(); i++) {
    a[i] = T(3 + i);
    b[i] = T(3 + 2 * i);
  }
  
  v = a;
  v += b;
  status = true;
  for (size_t i = 0; i < v.size(); i++) {
    T r = v[i];
    T x = a[i];
    T y = b[i];
    T z = x + y;
    if (r != z) status = false;
  }
  ts.check(status, "vector assignment addition");

  v = a;
  v -= b;
  status = true;
  for (size_t i = 0; i < v.size(); i++) {
    T r = v[i];
    T x = a[i];
    T y = b[i];
    T z = x - y;
    if (r != z) status = false;
  }
  ts.check(status, "vector assignment subtraction");

  v = a;
  v *= b;
  status = true;
  for (size_t i = 0; i < v.size(); i++) {
    T r = v[i];
    T x = a[i];
    T y = b[i];
    T z = x * y;
    if (r != z) status = false;
  }
  ts.check(status, "vector assignment multiplication");
  
  v = a;
  v /= b;
  status = true;
  for (size_t i = 0; i < v.size(); i++) {
    T r = v[i];
    T x = a[i];
    T y = b[i];
    T z = x / y;
    if (fabs(r - z) > 2 * gslmm::type_trait<float>::epsilon() * fabs(z)) 
      status = false;
  }
  ts.check(status, "vector assignment division");

  v = a + b;
  status = true;
  for (size_t i = 0; i < v.size(); i++) {
    T r = v[i];
    T x = a[i];
    T y = b[i];
    T z = x + y;
    if (r != z) status = false;
  }
  ts.check(status, "vector addition");

  v = a - b;
  status = true;
  for (size_t i = 0; i < v.size(); i++) {
    T r = v[i];
    T x = a[i];
    T y = b[i];
    T z = x - y;
    if (r != z) status = false;
  }
  ts.check(status, "vector subtraction");

#if 0
  v = a * b;
  status = true;
  for (size_t i = 0; i < v.size(); i++) {
    T r = v[i];
    T x = a[i];
    T y = b[i];
    T z = x * y;
    if (r != z) status = false;
  }
  ts.check(status, "vector multiplication");
  
  v = a / b;
  status = true;
  for (size_t i = 0; i < v.size(); i++) {
    T r = v[i];
    T x = a[i];
    T y = b[i];
    T z = x / y;
    if (fabs(r - z) > 2 * gslmm::type_trait<float>::epsilon() * fabs(z)) 
      status = false;
  }
  ts.check(status, "vector division");
#endif

}

  
//____________________________________________________________________  
/// Testing common operations
template <typename T>
void test_view(typename gslmm::vector<T>::size_type n) 
{
  gslmm::vector<T> v(2*n);
  for (size_t i = 0; i < v.size(); i++) v[i] = T(i);

  gslmm::vector<T> u(v, 0, n, 2);

  bool status = true;
  for (size_t i = 0; i < u.size(); i++) {
    if (u[i] != v[2*i]) 
      status = false;
  }
  ts.check(status, "view was correct");
  
  v[0] = 10;
  ts.test(u[0], 10, "view and orig points to same data");
  v[2*(n-1)] = 0;
  ts.test(u[n-1],0, "view and orig points to same data");
}

//____________________________________________________________________  
int main(int argc, char** argv) 
{
  std::cout << "vector-test.cc in" << std::endl;
  gslmm::test_suite ts1("vector", argc, argv);

  test_basic<double>(10);
  test_minmax<double>(10);
  test_operations<double>(10);
  test_view<double>(10);

  return 0;
}

//
// EOF
//  
