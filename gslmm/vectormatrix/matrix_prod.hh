//
// $Id: matrix_prod.hh,v 1.2 2006-06-05 09:20:10 cholm Exp $ 
//  
//  gslmm::matrix_prod
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
#ifndef GSLMM_vectormatrix_matrix_prod
#define GSLMM_vectormatrix_matrix_prod
#ifndef GSLMM_vectormatrix_matrix_base
# error This header is not for direct inclussion 
#endif
/** @file   vectormatrix/matrix_prod.hh
    @author Christian Holm
    @date   Mon Mar 10 14:50:12 2003
    @brief  Base template for products envolving matrices  */
#ifndef GSL_errno
# include <gsl/gsl_errno.h>
#endif

namespace gslmm
{
  //==================================================================
  template <typename Type> 
  inline void 
  matrix_matrix_product(const Type&         alpha, 
			const matrix<Type>& a,
			const matrix<Type>& b,
			const Type&         beta,
			matrix<Type>&       c)
  {
    c *= beta;
    for (size_t i = 0; i < a.row_size(); i++) {
      for (size_t j = 0; j < b.column_size(); j++) {
	Type temp = a(i, 0) * b(0, j);
	for (size_t k = 1; k < a.column_size(); k++) 
	  temp += a(i, k) * b(k, j);
	c(i,j) += alpha * temp;
      }
    }
  }
  //__________________________________________________________________
  template <typename Type> 
  inline void 
  matrix_matrix_product(const Type&               alpha, 
			const matrix_manip<Type>  a,
			const matrix_manip<Type>  b,
			const Type&               beta,
			matrix<Type>&             c)
  {
    c *= beta;
    for (size_t i = 0; i < a.row_size(); i++) {
      for (size_t j = 0; j < b.column_size(); j++) {
	Type temp = a(i, 0) * b(0, j);
	for (size_t k = 1; k < a.column_size(); k++) 
	  temp += a(i, k) * b(k, j);
	c(i,j) += alpha * temp;
      }
    }
  }
  //__________________________________________________________________
  template <typename Type> 
  inline void 
  matrix_matrix_product(const Type&               alpha, 
			const matrix_manip<Type>  a,
			const matrix<Type>&       b,
			const Type&               beta,
			matrix<Type>&             c)
  {
    c *= beta;
    for (size_t i = 0; i < a.row_size(); i++) {
      for (size_t j = 0; j < b.column_size(); j++) {
	Type temp = a(i, 0) * b(0, j);
	for (size_t k = 1; k < a.column_size(); k++) 
	  temp += a(i, k) * b(k, j);
	c(i,j) += alpha * temp;
      }
    }
  }
  //__________________________________________________________________
  template <typename Type> 
  inline void 
  matrix_matrix_product(const Type&               alpha, 
			const matrix<Type>&       a,
			const matrix_manip<Type>  b,
			const Type&               beta,
			matrix<Type>&             c)
  {
    c *= beta;
    for (size_t i = 0; i < a.row_size(); i++) {
      for (size_t j = 0; j < b.column_size(); j++) {
	Type temp = a(i, 0) * b(0, j);
	for (size_t k = 1; k < a.column_size(); k++) 
	  temp += a(i, k) * b(k, j);
	c(i,j) += alpha * temp;
      }
    }
  }

  //==================================================================
  template <typename Type>
  inline void 
  matrix_vector_product(const Type&         alpha, 
			const matrix<Type>& a, 
			const vector<Type>& v,
			const Type&         beta, 
			vector<Type>&       u)
  {
    if (a.column_size() != v.size()) 
      GSL_ERROR_VOID("matrix column size and vector size must match", 
		     GSL_EBADLEN);
    u *= beta;
    for (size_t i = 0; i < v.size(); i++) {
      Type temp = a(i, 0) * v[0];
      for (size_t j = 1; j < a.column_size(); j++) 
	temp += a(i, j) * v[j];
      u[i] += alpha * temp;
    }
  }
  //__________________________________________________________________
  template <typename Type>
  inline void 
  matrix_vector_product(const Type&               alpha, 
			const matrix_manip<Type>  a, 
			const vector<Type>&       v,
			const Type&               beta, 
			vector<Type>&             u)
  {
    if (a.column_size() != v.size()) 
      GSL_ERROR_VOID("matrix column size and vector size must match", 
		     GSL_EBADLEN);
    u *= beta;
    for (size_t i = 0; i < v.size(); i++) {
      Type temp = a(i, 0) * v[0];
      for (size_t j = 1; j < a.column_size(); j++) 
	temp += a(i, j) * v[j];
      u[i] += alpha * temp;
    }
  }
}

#endif
//____________________________________________________________________
//
// EOF
//
