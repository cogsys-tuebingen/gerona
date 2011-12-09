//
// $Id: matrix-test.cc,v 1.10 2006-05-11 00:39:39 cholm Exp $ 
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
#ifndef GSLMM_math_precision
#include <gslmm/math/type_trait.hh>
#endif
#ifndef GSLMM_vectormatrix_matrix
#include <gslmm/vectormatrix/matrix.hh>
#endif
#ifndef GSLMM_vectormatrix_matrix
#include <gslmm/vectormatrix/matrix_complex_double.hh>
#endif
#ifndef GSLMM_linear_lu_decomposition_double
# include <gslmm/linear/lu_decomposition_double.hh>
#endif
#ifndef GSLMM_blas_blas_double
# include <gslmm/blas/blas_double.hh>
#endif
#ifndef GSLMM_random_generator
#include <gslmm/random/generator.hh>
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

/** @file   matrix-test.cc
    @author Christian Holm
    @date   Mon Sep 16 13:03:50 2002
    @brief  Test of matrix classses.  */

//____________________________________________________________________
template <typename T>
bool
check_relative(const T& result, const T& expected, const T& error)
{
  bool ret = false;
  if (expected != 0) 
    ret = fabs(result - expected) / fabs(expected) <= error;
  else 
    ret = fabs(result) <= error; 
  return ret;
}

//____________________________________________________________________
template <typename T>
bool
check_relative(const gslmm::matrix<T>& r, gslmm::matrix<T>& e,
	       const T& eps, const char* msg) 
{
  int s = 0;
  for (size_t i = 0; i < r.row_size(); i++) 
    for (size_t j = 0; j < r.column_size(); j++) 
      if (!check_relative(r(i,j),e(i,j),eps)) s++;
  return ts.status(s, msg);
}

  
//____________________________________________________________________
void
test_complex()
{
  gslmm::matrix<gslmm::complex<double> > m(2,3);
  for (size_t row = 0; row < m.row_size(); row++) {
    for (size_t col = 0; col < m.column_size(); col++) {
      m(row,col) = gslmm::complex<double>(10*row, col);
    }
  }
  std::cout << "M\n" << m << std::endl;
  gslmm::matrix<gslmm::complex<double> > n(m,true);
  std::cout << "N\n" << n << std::endl;
  gslmm::matrix<gslmm::complex<double> > o(m,true,true);
  std::cout << "O\n" << o << std::endl;
}

    

//____________________________________________________________________  
template <typename T>
void make_random(gslmm::matrix<T>& a)
{
  gslmm::generator g;
  for (size_t row = 0; row < a.row_size(); row++) 
    for (size_t col = 0; col < a.column_size(); col++) 
      a(row, col) = g.uniform_nonzero();
}

//____________________________________________________________________  
template <typename T>
void test_mul_oper(const char* name) 
{
  T eps = gslmm::type_trait<T>::epsilon() * 1000;
  int s = 0;
  gslmm::matrix<T> A(1, 4);
  A(0,0) =  0.556; A(0,1) = -0.532; A(0,2) = -0.746; A(0,3) = -0.673;
  gslmm::matrix<T> B(4, 2);
  B(0,0) = -0.525; B(0,1) =  0.967; B(1,0) =  0.687; B(1,1) = -0.024; 
  B(2,0) =  0.527; B(2,1) =  0.485; B(3,0) =  0.109; B(3,1) = -0.46;
  gslmm::matrix<T> C(A * B);
  gslmm::matrix<T> R(1, 2);
  R(0,0) = -1.123883; R(0,1) = 0.49819;
  check_relative(C, R, eps, "gslmm::matrix<T> multiplication");
}
  
//____________________________________________________________________  
template <typename T>
void test_mul(const char* name) 
{
  // Try to calculate C=A*B
  T eps = gslmm::type_trait<T>::epsilon() * 1000;
  size_t d1 = 3, d2 = 4, d3 = 5;
  gslmm::matrix<T> A(d1, d2);
  gslmm::matrix<T> B(d2, d3);
  make_random(A);
  make_random(B);
  gslmm::matrix<T> C(A * B);
  gslmm::matrix<T> D(d1, d3);
  D = A * B;
  int s = 0;
  check_relative(C, D, eps, "gslmm::matrix<T> multiplication");
}

//____________________________________________________________________  
template <typename T>
void test_mul3(const char* name) 
{
  // Try to calculate C=A*B
  T eps = gslmm::type_trait<T>::epsilon() * 1000;
  size_t d = 3;
  gslmm::matrix<T> A(d, d);
  gslmm::matrix<T> B(d, d);
  gslmm::matrix<T> C(d, d);
  gslmm::matrix<T> D(d, d);
  A(0,0) = A(1,1) = A(2,2) = 1;
  B(0,0) = B(1,1) = B(2,2) = 1;
  D(0,0) = D(1,1) = D(2,2) = 1;
  make_random(C);
  gslmm::matrix<T> E(d, d);
  gslmm::matrix<T> F(d, d);
  E = (A * B) * C;
  check_relative(E, C, eps, "gslmm::matrix<T> multiple multiplication");
  F = (A * B) * C * D;
  F = (A * B * C) * D;
  F = A * (B * C) * D;
  F = A * (B * C * D);
  F = (A * B) * (C * B);
  check_relative(F, C, eps, "gslmm::matrix<T> multiple multiplication");
}

//____________________________________________________________________  
template <typename T>
void test_vmul(const char* name) 
{
  // Try to calculate C=A*B
  int    s   = 0;
  T      eps = gslmm::type_trait<T>::epsilon() * 1000;
  size_t d   = 3;
  gslmm::matrix<T> A(d, d);
  gslmm::matrix<T> B(d, d);
  gslmm::matrix<T> C(d, d);
  A(0,0) = A(1,1) = A(2,2) = 1;
  B(0,0) = B(1,1) = B(2,2) = 1;
  C(0,0) = C(1,1) = C(2,2) = 1;
  gslmm::vector<T> v(d);
  v[0] = 1; v[1] = 2; v[2] = 3;
  gslmm::vector<T> u(A * v);
  gslmm::vector<T> w(u.size());

  w = A * v;
  s = 0;
  for (size_t i = 0; i < d; i++) if (!check_relative(u[i],w[i], eps)) s++;
  ts.status(s, "matrix<%s>-vector<%s> multiplication", name, name);

  w = (A * B) * v;
  s = 0;
  for (size_t i = 0; i < d; i++) if (!check_relative(u[i],w[i], eps)) s++;
  ts.status(s, "matrix<%s>-vector<%s> multiplication", name, name);

  w = A * (B * v);
  s = 0;
  for (size_t i = 0; i < d; i++) if (!check_relative(u[i],w[i], eps)) s++;
  ts.status(s, "matrix<%s>-vector<%s> multiplication", name, name);

  w = (A * B) * (C * v);
  s = 0;
  for (size_t i = 0; i < d; i++) if (!check_relative(u[i],w[i], eps)) s++;
  ts.status(s, "matrix<%s>-vector<%s> multiplication", name, name);
  
}

//____________________________________________________________________  
template <typename T>
void test_div22(const char* name) 
{
  T eps = gslmm::type_trait<T>::epsilon() * 1000;
  gslmm::matrix<T> A(2,2);
  gslmm::matrix<T> B(2,2);
  A(0,0) = A(1,1) = 1;
  B.set(1.); B(1,1) = -1;
  gslmm::matrix<T> R(2,2);
  R.set(.5); R(1,1) = -.5;
  gslmm::matrix<T> C(A / B);
  check_relative(C, R, eps, "A / B");
  R = B * C;
  check_relative(R, A, eps, "B A / B");
}

  
#if 0  
//____________________________________________________________________  
template <typename T>
void test_div23(const char* name) 
{
  // Try to calculate C=A*B
  size_t d1 = 2, d2 = 3;
  T eps = gslmm::type_trait<T>::epsilon() * 1000;
  gslmm::matrix<T> A(d1,d2);
  gslmm::matrix<T> B(d1,d2);
  B.set(1.);
  B(d1-1,d2-1) = -1;
  A(0,0) = A(1,1) = 1;
  gslmm::matrix<T> C(A / B);
  gslmm::matrix<T> D(C * B);
  int s = 0;
  std::cout << "A/B:\n" << C << std::endl;
  std::cout << "B * A/B:\n" << D << std::endl;
  for (size_t i = 0; i < d1; i++) 
    for (size_t j = 0; j < d2; j++) 
      if (!ts.relative(D(i,j),A(i,j), eps, "B A / B [%d,%d]", i, j))
	s++;
  ts.status(s, "matrix<%s>-matrix<%s> right division (2x3)", name, name);
  gslmm::matrix<T> Bt(B, true);
  gslmm::qr_decomposition<T> qr(Bt);
  gslmm::matrix<T> Binv(d2,d2);
  for (size_t i = 0; i < Bt.row_size(); i++) {
    gslmm::vector<T> v(Bt.row_size());
    gslmm::vector<T> x(Bt.column_size());
    gslmm::vector<T> r(Bt.row_size());
    v.basis(i);
    qr.solve(v, x, r);
    std::cout << v << " " << x << " " << r << std::endl;
  }
}
#endif
  
  


//____________________________________________________________________  
template <typename T>
void test_basic(size_t rows, size_t columns, const char* name) 
{
  gslmm::matrix<T> m(rows, columns);

  ts.test(m.row_size(), rows, "gslmm::matrix<%s>::matrix "
	  "returns valid row size", name);
  ts.test(m.column_size(), columns, "gslmm::matrix<%s>::matrix "
	  "returns valid column size", name);
  ts.test(m.tda(), columns, "gslmm::matrix<%s>::matrix "
	  "returns valid tda", name);
}


//____________________________________________________________________  
template <typename T>
void test_access(size_t rows, size_t columns, const char* name) 
{
  bool status = true;
  gslmm::matrix<T> m(rows, columns);

  size_t k = 0;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      k++;
      m(i,j) = T(k);
    }
  }

  status = true;
  k = 0;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      k++;
      if (m(i,j) != T(k))
	status = false;
    }
  }
  ts.check(status, "gslmm::matrix<%s>::operator() writes into array",name);

  m.zero();
  k = 0;
  for (size_t i = 0; i < m.row_size(); i++)  {
    for (size_t j = 0; j < m.column_size(); j++) {
      k++;
      m[i][j] = T(k);
    }
  }

  status = true;
  k = 0;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      k++;
      if (m(i, j) != T(k))
	status = false;
    }
  }
  ts.check(status, "gslmm::matrix<%s>::operator[][] writes into array",name);

  status = true;
  k = 0;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      k++;
      if (m[i][j] != T(k))
	status = false;
    }
  }
  ts.check(status, "gslmm::matrix<%s>::operator[][] reads from array",name);



  status = true;
  k = 0;
  for (size_t i = 0; i < m.row_size(); i++) {
    gslmm::vector<T> v = m[i];
    for (size_t j = 0; j < m.column_size(); j++) {
      k++;
      if (v[j] != T(k))
	status = false;
    }
  }
  ts.check(status, "gslmm::matrix<%s>::operator[] extracts row",name);
}


//____________________________________________________________________  
template <typename T>
void test_minmax(size_t rows, size_t columns, const char* name) 
{
  gslmm::matrix<T> m(rows, columns);

  size_t k = 0;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      k++;
      m(i,j) = T(k);
    }
  }

  T      exp_max  = m(0, 0);
  T      exp_min  = m(0, 0);
  size_t exp_imax = 0;
  size_t exp_jmax = 0;
  size_t exp_imin = 0;
  size_t exp_jmin = 0;

  for (size_t i = 0; i < m.row_size(); i++){
    for (size_t j = 0; j < m.column_size(); j++){
      T k = m[i][j];
      if (k > exp_max) {
	exp_max  = m(i,j);
	exp_imax = i;
	exp_jmax = j;
      }
      if (k < exp_min) {
	exp_min  =  m(i, j);
	exp_imin = i;
	exp_jmin = j;
      }
    }
  }
  T max = m.max();
  T min = m.min();
  ts.test(max, exp_max, "gslmm::matrix<%s>::max returns maximum value",name);
  ts.test(min, exp_min, "gslmm::matrix<%s>::min returns minimum value",name);

  m.minmax(min,max);
  ts.test(max, exp_max, "gslmm::matrix<%s>::minmax "
	  "returns maximum value",name);
  ts.test(min, exp_min, "gslmm::matrix<%s>::minmax " 
	  "returns minimum value",name);

  size_t imax = 0;
  size_t jmax = 0;
  size_t imin = 0;
  size_t jmin = 0;
  m.max_index(imax,jmax);
  ts.test(imax, exp_imax, "gslmm::matrix<%s>::max_index "
	  "returns maximum i",name);
  ts.test(jmax, exp_jmax, "gslmm::matrix<%s>::max_index "
	  "returns maximum j",name);

  m.min_index(imin,jmin);
  ts.test(imin, exp_imin, "gslmm::matrix<%s>::min_index returns "
	  "minimum i",name);
  ts.test(jmin, exp_jmin, "gslmm::matrix<%s>::min_index "
	  "returns minimum j",name);

  m.minmax_index(imin,jmin,imax,jmax);
  ts.test(imax, exp_imax, "gslmm::matrix<%s>::minmax_index "
	  "returns maximum i",name);
  ts.test(jmax, exp_jmax, "gslmm::matrix<%s>::minmax_index "
	  "returns maximum j",name);
  ts.test(imin, exp_imin, "gslmm::matrix<%s>::minmax_index "
	  "returns minimum i",name);
  ts.test(jmin, exp_jmin, "gslmm::matrix<%s>::minmax_index "
	  "returns minimum j",name);
}


//____________________________________________________________________  
template <typename T>
void test_operations(size_t rows, size_t columns, const char* name) 
{
  bool status = true;

  gslmm::matrix<T> m(rows, columns);

  size_t k = 0;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      k++;
      m(i,j) = T(k);
    }
  }
  gslmm::matrix<T> a(m.row_size(), m.column_size());
  gslmm::matrix<T> b(m.row_size(), m.column_size());
    
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      a(i, j) = T(3 + i +  5 * j);
      b(i, j) = T(3 + 2 * i + 4 * j);
    }
  }
    
  m = a;
  m += b;
  status = true;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      T r = m(i,j);
      T x = a(i,j);
      T y = b(i,j);
      T z = x + y;
      if (r != z) status = false;
    }
  }
  ts.check(status, "gslmm::matrix<%s>::operator+= matrix addition",name);

  m =  a;
  m -= b;
  
  status = true;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      T r = m(i,j);
      T x = a(i,j);
      T y = b(i,j);
      T z = x - y;
      if (r != z) status = false;
    }
  }
  ts.check(status, "gslmm::matrix<%s>::operator-= matrix subtraction",name);

  m =  a;
  m *= b;
    
  status = true;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      T r = m(i,j);
      T x = a(i,j);
      T y = b(i,j);
      T z = x * y;
      if (r != z)
	status = false;
    }
  }
  ts.check(status, "gslmm::matrix<%s>::operator*= multiplication",name);

  m =  a;
  m /= b;

  status = true;
  for (size_t i = 0; i < m.row_size(); i++) {
    for (size_t j = 0; j < m.column_size(); j++) {
      T r = m(i,j);
      T x = a(i,j);
      T y = b(i,j);
      T z = x / y;
      if (fabs(r - z) > 2 * gslmm::type_trait<float>::epsilon() * fabs(z))
	status = false;
    }
  }
  ts.check(status, "gslmm::matrix<%s>::operator/= division",name);
}

//____________________________________________________________________  
int main(int argc, char** argv) 
{
  gslmm::test_suite ts1("matrix", argc, argv);

  // test_complex();
  // test_mul<double>("double");
  test_mul3<double>("double");
  test_vmul<double>("double");
  // test_mul_oper<double>("double");
  //test_div22<double>("double");
  // test_div23<double>("double");
  //test_basic<double>(5,10, "double");
  //test_access<double>(5,10, "double");
  //test_minmax<double>(5,10, "double");
  //test_operations<double>(5,10, "double");
  return ts.summary() ? 0 : 1;
}

//
// EOF
//  
