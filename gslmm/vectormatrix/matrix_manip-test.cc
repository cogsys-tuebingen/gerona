//
// $Id: matrix_manip-test.cc,v 1.1 2006-05-01 14:23:36 cholm Exp $ 
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
#ifndef GSLMM_compleks_complex_double
#include <gslmm/compleks/complex.hh>
#endif
#ifndef GSLMM_vectormatrix_matrix
#include <gslmm/vectormatrix/matrix.hh>
#endif
#ifndef GSLMM_vectormatrix_matrix_manip
#include <gslmm/vectormatrix/matrix_manip.hh>
#endif
#ifndef GSLMM_random_generator
#include <gslmm/random/generator.hh>
#endif
#ifndef GSLMM_blas_generator
#include <gslmm/blas/blas.hh>
#endif
#ifndef GSLMM_test_suite
#include <gslmm/test/test_suite.hh>
#endif
#ifndef __IOMANIP__
#include <iomanip>
#endif
#define ts2 gslmm::test_suite::instance()
gslmm::generator g;

//____________________________________________________________________  
template <typename T>
void print_row(const T& a, size_t i, size_t w) 
{
  size_t       ncol = a.column_size();
  // std::cout << std::fixed;
  for (size_t j = 0; j < ncol; j++) {
    if  (i >= a.row_size()) 
      std::cout << std::setw(w) << " ";
    else 
      std::cout << std::setw(w) << a(i, j);
    std::cout << " ";
  }
  std::cout << std::flush;
}


//____________________________________________________________________  
template <typename T1, typename T2, typename T3>
void print_3(const T1& a, const T2& b, const T3& c, size_t w)
{
  size_t nrow = std::max(std::max(a.row_size(), b.row_size()), c.row_size());
  for (size_t i = 0; i < nrow; i++) {
    char lchar = (i == 0 ? '/'  : (i == nrow-1 ? '\\' : '|'));
    char rchar = (i == 0 ? '\\' : (i == nrow-1 ? '/'  : '|'));
    std::cout << lchar;
    print_row(a, i, w);
    std::cout << rchar << "   " << lchar;
    print_row(b, i, w);
    std::cout << rchar << "   " << lchar;
    print_row(c, i, w);
    std::cout << rchar << std::endl;
  }
}

//____________________________________________________________________  
template <typename T1, typename T2>
void print_2(const T1& a, const T2& b, size_t w)
{
  size_t nrow = std::max(a.row_size(), b.row_size());
  for (size_t i = 0; i < nrow; i++) {
    char lchar = (i == 0 ? '/'  : (i == nrow-1 ? '\\' : '|'));
    char rchar = (i == 0 ? '\\' : (i == nrow-1 ? '/'  : '|'));
    std::cout << lchar;
    print_row(a, i, w);
    std::cout << rchar << "   " << lchar;
    print_row(b, i, w);
    std::cout << rchar << std::endl;
  }
}

//____________________________________________________________________  
template <typename T>
void make_random(gslmm::matrix<T>& a);

//____________________________________________________________________  
template <>
void make_random(gslmm::matrix<double>& a)
{
  for (size_t row = 0; row < a.row_size(); row++) 
    for (size_t col = 0; col < a.column_size(); col++) 
      a(row, col) =  g.uniform_nonzero();
}

//____________________________________________________________________  
template <>
void make_random(gslmm::matrix<gslmm::complex<double> >& a)
{
  for (size_t row = 0; row < a.row_size(); row++) 
    for (size_t col = 0; col < a.column_size(); col++) 
      a(row, col) =  gslmm::complex<double>(g.uniform_nonzero(),
					    (row == col ? 0 : 
					     g.uniform_nonzero()));
}

//____________________________________________________________________  
template <typename T>
void make_symmetric(gslmm::matrix<T>& a, bool upper=true)
{
  for (size_t i = 0; i < a.row_size(); i++) 
    for (size_t j = 0; j < a.column_size(); j++) 
      if (upper && j < i)       a(i,j) = a(j,i);
      else if (!upper && j > i) a(i,j) = a(j,i);
}

//____________________________________________________________________  
template <typename T>
void make_hermitian(gslmm::matrix<T>& a, bool upper=true)
{
  typedef typename gslmm::matrix<T>::trait_type trait_type;
  for (size_t i = 0; i < a.row_size(); i++) 
    for (size_t j = 0; j < a.column_size(); j++) 
      if (upper && j < i)       a(i,j) = trait_type::conjugate(a(j,i));
      else if (!upper && j > i) a(i,j) = trait_type::conjugate(a(j,i));
}

//____________________________________________________________________  
template <typename T>
void make_triangular(gslmm::matrix<T>& a, bool upper=true)
{
  for (size_t i = 0; i < a.row_size(); i++) 
    for (size_t j = 0; j < a.column_size(); j++) 
      if (upper && j < i)       a(i,j) = 0;
      else if (!upper && j > i) a(i,j) = 0;
}

//____________________________________________________________________  
template <typename T>
void
make_matrix(gslmm::matrix<T>& a);

//____________________________________________________________________  
void
make_matrix(gslmm::matrix<double>& a) 
{
  for (size_t row = 0; row < a.row_size(); row++) 
    for (size_t col = 0; col < a.column_size(); col++) 
      a(row, col) =  (row+1) * 10 + (col + 1);
}

//____________________________________________________________________  
void
make_matrix(gslmm::matrix<gslmm::complex<double> >& a) 
{
  for (size_t row = 0; row < a.row_size(); row++) 
    for (size_t col = 0; col < a.column_size(); col++) 
      a(row, col) =  gslmm::complex<double>((row+1),
					    (row == col ? 0 : (col + 1)));
}


//____________________________________________________________________  
template <typename T>
gslmm::matrix<T>  
make_transformed(gslmm::matrix<T>& a,
		 gslmm::type       u=gslmm::general_type,
		 gslmm::transform  t=gslmm::no_transform, 
		 gslmm::location   l=gslmm::upper_triangle, 
		 gslmm::diagonal   d=gslmm::non_unit_diagonal, 
		 gslmm::side       s=gslmm::right_side)
{
  typedef gslmm::matrix<T> matrix_type;
  typedef typename gslmm::matrix<T>::trait_type trait_type;
  matrix_type id(a.row_size(), a.column_size());
  for (size_t i = 0; i < a.row_size(); i++) 
    id(i,i) = trait_type::unit();
  matrix_type b(a.row_size(), a.column_size());
  gslmm::matrix_manip<T> at(a, u, t, l, d, s);
  gslmm::matrix_matrix_product(trait_type::unit(),at,id,trait_type::null(),b);
#if 0  
  matrix_type b(a, (u == gslmm::general_type && t != gslmm::no_transform));
  if (u == gslmm::triangular_type) {
    make_triangular(b, l == gslmm::upper_triangle);
    if (t != gslmm::no_transform) b.transpose();
    if (d == gslmm::unit_diagonal) 
      for (size_t i = 0; i < b.row_size() && i < b.column_size(); i++)
	b(i,i) = 1;
  }
  else if (u == gslmm::symmetric_type) 
    make_symmetric(b,  l == gslmm::upper_triangle);
  else if (u == gslmm::hermitian_type) 
    make_hermitian(b,  l == gslmm::upper_triangle);
#endif
  return b;
}

    
//____________________________________________________________________  
std::string manip_str(gslmm::type      u=gslmm::general_type,
		      gslmm::transform t=gslmm::no_transform, 
		      gslmm::location  l=gslmm::upper_triangle, 
		      gslmm::diagonal  d=gslmm::non_unit_diagonal, 
		      gslmm::side      s=gslmm::right_side)
{
  using namespace gslmm;
  std::stringstream str;
  switch (u) {
  case general_type:             str << "Gen"; break;
  case triangular_type:          str << "Tri"; break;
  case inverse_triangular_type:  str << "Tri"; break;
  case symmetric_type:           str << "Sym"; break;
  case hermitian_type:           str << "Her"; break;
  }
  if (u != general_type) {
    switch (l) {
    case upper_triangle:         str << " Upper"; break;
    case lower_triangle:         str << " Lower"; break;
    }
    switch (d) {
    case non_unit_diagonal:      str << "          "; break;
    case unit_diagonal:          str << " diag(A)=1"; break;
    }
  }
  else                                  str << "                ";
  str << ": A";
  switch (t) {
  case no_transform:             str << "  "; break;
  case transpose_transform:      str << "^T"; break;
  case adjoint_transform:        str << "^H"; break;
  }
  return str.str();
}


//____________________________________________________________________  
template <typename T>
void test_manip(const char* name,
		gslmm::type      u=gslmm::general_type,
		gslmm::transform t=gslmm::no_transform, 
		gslmm::location  l=gslmm::upper_triangle, 
		gslmm::diagonal  d=gslmm::non_unit_diagonal, 
		gslmm::side      s=gslmm::right_side)
{
  gslmm::matrix<T>       A(3,3);
  make_matrix(A);
  gslmm::matrix<T>       R(make_transformed(A, u, t, l, d, s));
  gslmm::matrix_manip<T> TA(A, u, t, l, d, s);
  std::string str(manip_str(u, t, l, d, s));
  int se = 0;
  for (size_t i = 0; i < A.row_size(); i++) 
    for (size_t j = 0; j < A.row_size(); j++) 
      if (TA(i,j) != R(i,j)) se++;

  if (!ts2.status(se, "matrix_manip<%s>: %s", name, str.c_str())
      && ts2.is_verbose()) print_3(A, TA, R, 4);
}

//____________________________________________________________________  
template <typename T>
void test_all_manip(const char* name)
{
  using namespace gslmm;
  type us[] = { general_type, symmetric_type, hermitian_type, triangular_type};
  for (size_t iu = 0; iu < 3; iu++) {
    transform ts[] = { no_transform, transpose_transform };
    for (size_t it = 0; it < 2; it++) {
      if (us[iu] == general_type) {
	test_manip<T>(name, us[iu], ts[it]);
	continue;
      }
      location ls[] = { upper_triangle, lower_triangle };
      for (size_t il = 0; il < 2; il++) {
	diagonal ds[] = { non_unit_diagonal, unit_diagonal };
	for (size_t id = 0; id < 2; id++) {
	  test_manip<T>(name, us[iu], ts[it], ls[il], ds[id]);
	}
      }
    }
  }
}

//____________________________________________________________________  
template <typename T>
int 
compare(const char* name, const char* dsc,
	const gslmm::matrix<T>& a, 
	const gslmm::matrix<T>& b);


//____________________________________________________________________  
template <>
int 
compare(const char* name, const char* dsc,
	const gslmm::matrix<double>& a, 
	const gslmm::matrix<double>& b) 
{
  int se = 0;
  const double& eps  = gslmm::type_trait<double>::epsilon();
  const double  tol  = 100 * eps;
  for (size_t i = 0; i < a.row_size(); i++) 
    for (size_t j = 0; j < a.row_size(); j++) 
      if (!ts2.absolute(a(i,j),b(i,j),tol, 
		       "matrix_manip<%s> mult: %s (%d,%d)", name, dsc, i, j)) 
	se++;  
  return se;
}

//____________________________________________________________________  
template <>
int 
compare(const char* name, const char* dsc,
	const gslmm::matrix<gslmm::complex<double> >& a, 
	const gslmm::matrix<gslmm::complex<double> >& b) 
{
  int se = 0;
  const double& eps  = gslmm::type_trait<double>::epsilon();
  const double  tol  = 0.000001;
  for (size_t i = 0; i < a.row_size(); i++) {
    for (size_t j = 0; j < a.row_size(); j++) {
      if (!ts2.absolute(a(i,j).real(),b(i,j).real(),tol, 
		       "matrix_manip<%s> mult: %s Real(%d,%d)", 
			name, dsc, i, j)) se++;  
      if (!ts2.absolute(a(i,j).imag(),b(i,j).imag(),tol, 
		       "matrix_manip<%s> mult: %s Imag(%d,%d)", 
			name, dsc, i, j)) se++;  
    }
  }
  return se;
}


//____________________________________________________________________  
template <typename T>
void test_mult_manip(const char* name,
		     gslmm::type      u=gslmm::general_type,
		     gslmm::transform t=gslmm::no_transform, 
		     gslmm::location  l=gslmm::upper_triangle, 
		     gslmm::diagonal  d=gslmm::non_unit_diagonal, 
		     gslmm::side      s=gslmm::left_side)
{
  using namespace gslmm;
  const T& unit = type_trait<T>::unit();
  const T& null = type_trait<T>::null();
  matrix<T> A(2,2);
  matrix<T> B(2,2);
  matrix<T> R(2,2);
  make_random(A);
  make_random(B);
  matrix_manip<T> TA(A, u, t, l, d, s);
  matrix<T> S(make_transformed(A,u,t,l,d,s));
  matrix<T> C(TA * B);
  matrix_matrix_product(unit, TA, B, null, R);
  std::string str(manip_str(u,t,l,d,s));
  int se = 0;
  for (size_t i = 0; i < A.row_size(); i++) 
    for (size_t j = 0; j < A.row_size(); j++) 
      se += compare(name, str.c_str(), C, R);
  if (!ts2.status(se, "matrix_manip<%s> mult: %s", name, str.c_str())
      && ts2.is_verbose()) { 
    print_2(TA, B, 10);
    print_2(C, R, 10);
  }
}

  
//____________________________________________________________________  
template <typename T>
void test_all_manip_mult(const char* name)
{
  using namespace gslmm;
  type us[] = { general_type, symmetric_type, hermitian_type, 
		triangular_type };
  for (size_t iu = 0; iu < 3; iu++) {
    transform ts[] = { no_transform, transpose_transform };
    for (size_t it = 0; it < 2; it++) {
      if (us[iu] == general_type) {
	test_mult_manip<T>(name, us[iu], ts[it]);
	// continue;
	break;
      }
      location ls[] = { upper_triangle, lower_triangle };
      for (size_t il = 0; il < 2; il++) {
	diagonal ds[] = { non_unit_diagonal, unit_diagonal };
	for (size_t id = 0; id < 2; id++) {
	  test_mult_manip<T>(name, us[iu], ts[it], ls[il], ds[id]);
	}
      }
    }
  }
}

  
//____________________________________________________________________  
int main(int argc, char** argv) 
{
  
  gslmm::test_suite ts1("matrix", argc, argv);
  test_all_manip<double>("double");
  test_all_manip<gslmm::complex<double> >("complex<double>");
  test_all_manip_mult<double>("double");
  test_all_manip_mult<gslmm::complex<double> >("complex<double>");
  
  return ts1.summary() ? 0 : 1;
}

//
// EOF
//  

