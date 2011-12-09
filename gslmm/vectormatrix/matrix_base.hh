//
// $Id: matrix_base.hh,v 1.19 2007-04-21 09:03:14 cholm Exp $ 
//  
//  gslmm::matrix_base
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
#ifndef GSLMM_vectormatrix_matrix_base
#define GSLMM_vectormatrix_matrix_base
#ifndef GSLMM_vectormatrix_vector_base
# include <gslmm/vectormatrix/vector_base.hh>
#endif
#ifndef GSL_errno
# include <gsl/gsl_errno.h>
#endif
#include <cassert>

/** @file   vectormatrix/matrix_base.hh
    @author Christian Holm
    @date   Mon Mar 10 14:50:12 2003
    @brief  Base template for matricies classes */

namespace gslmm
{
  /** @class matrix matrix_base.hh <gslmm/vectormatrix/matrix_base.hh>
      @brief Base template for matricies classes.
      @ingroup matrix
   */
  template <typename Type>
  class matrix;

  // Forward declarations 
  template <typename T> 
  struct matrix_manip;

  //================================================================
  /** @brief Structure holding an operation that results in a matrix
   */
  template <typename T> 
  struct matrix_return
  {
    /** Apply operator. 
	@param a On return, is assigned to the result of the
	operation */
    virtual void operator()(matrix<T>& a) const = 0;
    /** @return Row size of the resulting matrix */
    virtual size_t row_size() const = 0;
    /** @return Column size of the resulting matrix */
    virtual size_t column_size() const = 0;
    /** Destructor */
    virtual ~matrix_return() {}
  };

  //==================================================================
  template <typename T> 
  struct matrix_oper
  {
    typedef matrix<T>                  matrix_type;
    typedef vector<T>                  vector_type;
    typedef matrix_manip<T>            manip_type;
    typedef matrix_return<T>           return_type;
    typedef std::auto_ptr<return_type> ptr_type;
  };

  //================================================================
  /** @brief Trait of matrix-matrix operation 
   */
  template <typename T, template <class> class Arg>
  struct m_arg_trait;

  //________________________________________________________________
  /** @brief Trait of matrix-matrix operation, specialised for
      matrix<T> 
  */
  template <typename T>
  struct m_arg_trait<T, matrix>
  {
    /** Type of matrix */
    typedef matrix<T> matrix_type;
    /** Type of argument to ctor */
    typedef const matrix<T>& ctor_type;
    /** Stored type */
    typedef const matrix<T>& member_type;
    /** type in @c operator* */
    typedef const matrix<T>& oper_type;
    /** type in @c doit argument */
    typedef const matrix<T>& doit_type;
    /** Assert that @a m is valid */
    static void check(ctor_type ) {}
    /** Get the row size of @a m */
    static size_t row_size(member_type m) { return m.row_size(); }
    /** Get the column size of @a m */
    static size_t column_size(member_type m) { return m.column_size(); }
    /** Apply and destroy */
    static void apply(doit_type m, matrix_type& c) { c = m; }
    enum { temp = 0  };
  };

  //________________________________________________________________
  /** @brief Trait of matrix-matrix operation, specialised for
      matrix_manip<T> 
  */
  template <typename T>
  struct m_arg_trait<T, matrix_manip>
  {
    /** Type of matrix */
    typedef matrix<T> matrix_type;
    /** Type of argument to ctor */
    typedef const matrix_manip<T>& ctor_type;
    /** Stored type */
    typedef const matrix_manip<T>  member_type;
    /** type in @c operator* */
    typedef const matrix_manip<T>  oper_type;
    /** type in @c doit argument */
    typedef const matrix_manip<T>& doit_type;
    /** Assert that @a m is valid */
    static void check(ctor_type) {}
    /** Get the row size of @a m */
    static size_t row_size(member_type m) { return m.row_size(); }
    /** Get the column size of @a m */
    static size_t column_size(member_type m) { return m.column_size(); }
    /** Apply and destroy */
    static void apply(doit_type m, matrix_type& c) { c = m; }
    enum { temp = 0 };
  };

  //________________________________________________________________
  /** @brief Trait of matrix-matrix operation, specialised for
      matrix_manip<T> 
  */
  template <typename T>
  struct m_arg_trait<T, matrix_return>
  {
    /** Type of matrix */
    typedef matrix<T> matrix_type;
    /** Type of argument to ctor */
    typedef typename matrix_oper<T>::ptr_type ctor_type;
    /** Stored type */
    typedef typename matrix_oper<T>::ptr_type member_type;
    /** type in @c operator* */
    typedef typename matrix_oper<T>::ptr_type oper_type;
    /** type in @c doit argument */
    typedef const typename matrix_oper<T>::ptr_type& doit_type;
    /** Assert that @a m is valid */
    static void check(doit_type m) { assert(m.get());  }
    /** Get the row size of @a m */
    static size_t row_size(doit_type m) { return m->row_size(); }
    /** Get the column size of @a m */
    static size_t column_size(doit_type m){return m->column_size();}
    /** Apply and destroy */
    static void apply(doit_type m, matrix_type& c) { m->operator()(c); }
    enum { temp = 1 };
  };
  
  
  //==================================================================
  /** @{
      @name matrix-matrix products @f$ C = A B@f$ */
  /** * operation between matricies @f$ C = A B@f$
      The implementation is straight forward.  No hanky-panky here. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand @f$ A @f$
      @param rhs The right hand operand @f$ B @f$
      @return new matrix, @f$ C_{ij} = \sum_k A_{ik} B_{kj} @f$ */
  template <typename T>
  typename matrix_oper<T>::ptr_type 
  operator*(const matrix<T>& lhs, const matrix<T>& rhs);

  //__________________________________________________________________
  /** * operation between matricies  @f$ C = A B@f$
      The implementation is straight forward.  No hanky-panky here. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand @f$ A @f$
      @param rhs The right hand operand @f$ op_B(B) @f$
      @return new matrix, @f$ C_{ij} = \sum_k A_{ik} op_B(B_{kj}) @f$ */
  template <typename T>
  typename matrix_oper<T>::ptr_type 
  operator*(const matrix<T>& lhs, const matrix_manip<T> rhs);

  //__________________________________________________________________
  /** * operation between matrices  @f$ C = A B@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates products like @f$ C = A (D E)@f$, by
      operating on the result of @f$ (D E)@f$ as the right hand side
      @f$ B@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @internal It @e has to be @c std::auto_ptr<matrix_return<T>>
      here - using @c typename @c matrix_oper<T>::ptr_type does not
      work 
      @param lhs The left hand operand @f$ A @f$
      @param rhs The right hand operand @f$ B @f$ - the result of a
      previous operation returing a matrix.   
      @return new matrix, @f$ C_{ij} = \sum_k A_{ik} B_{kj} @f$ */
  template <typename T>
  typename matrix_oper<T>::ptr_type 
  operator*(const matrix<T>& lhs, std::auto_ptr<matrix_return<T> > rhs);
  // typename matrix_oper<T>::ptr_type);

  //__________________________________________________________________
  /** * operation between matrices @f$ C = A B@f$
      The implementation is straight forward.  No hanky-panky here. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand @f$ A @f$
      @param rhs The right hand operand @f$ B @f$
      @return new matrix, @f$ C_{ij} = \sum_k op_A(A)_{ik} B_{kj} @f$ */
  template <typename T>
  typename matrix_oper<T>::ptr_type 
  operator*(const matrix_manip<T> lhs, const matrix<T>& rhs);

  //__________________________________________________________________
  /** * operation between matrices @f$ C = A B@f$
      The implementation is straight forward.  No hanky-panky here. 
      @ingroup matrix
      @param lhs The left hand operand @f$ A @f$
      @param rhs The right hand operand @f$ B @f$
      @return new matrix, @f$ C_{ij} = \sum_k op_A(A)_{ik} op_B(B)_{kj} @f$ */
  template <typename T>
  typename matrix_oper<T>::ptr_type 
  operator*(const matrix_manip<T> lhs, const matrix_manip<T> rhs);  

  //__________________________________________________________________
  /** * operation between matrices @f$ C = A B@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates products like @f$ C = A (D E)@f$, by
      operating on the result of @f$ (D E)@f$ as the right hand side
      @f$ B@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @internal It @e has to be @c std::auto_ptr<matrix_return<T>>
      here - using @c typename @c matrix_oper<T>::ptr_type does not
      work 
      @ingroup matrix
      @param lhs The left hand operand @f$ A @f$
      @param rhs The right hand operand @f$ B @f$ - the result of a
      previous operation returing a matrix.   
      @return new matrix, @f$ C_{ij} = \sum_k op_A(A)_{ik} B_{kj} @f$ */
  template <typename T>
  typename matrix_oper<T>::ptr_type 
  operator*(const matrix_manip<T> lhs, std::auto_ptr<matrix_return<T> > rhs);

  //__________________________________________________________________
  /** * operation between matricies @f$ C = A B@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates products like @f$ C = (D E) B@f$, by
      operating on the result of @f$ (D E)@f$ as the left hand side
      @f$ A@f$.
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand @f$ A @f$ - the result of a
      previous operation returing a matrix.   
      @param rhs The right hand operand @f$ B @f$
      @return new matrix, @f$ C_{ij} = \sum_k A_{ik} B_{kj} @f$ */
  template <typename T>
  typename matrix_oper<T>::ptr_type 
  operator*(typename matrix_oper<T>::ptr_type lhs, const matrix<T>& rhs);

  //__________________________________________________________________
  /** * operation between matrices @f$ C = A B@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates products like @f$ C = (D E) B@f$, by
      operating on the result of @f$ (D E)@f$ as the left hand side
      @f$ A@f$.
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand @f$ A @f$ - the result of a
      previous operation returing a matrix.   
      @param rhs The right hand operand @f$ op_B(B) @f$
      @return new matrix, @f$ C_{ij} = \sum_k A_{ik} op_B(B)_{kj} @f$ */
  template <typename T>
  typename matrix_oper<T>::ptr_type 
  operator*(typename matrix_oper<T>::ptr_type lhs, const matrix_manip<T> rhs);  

  //__________________________________________________________________
  /** * operation between matrices @f$ C = A B@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates products like @f$ C = (D E) (F G)@f$, by
      operating on the result of @f$ (D E)@f$ as the left hand side
      @f$ A@f$, and @f$ (F G)@f$ as the right hand side  @f$ B@f$.
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @internal It @e has to be @c std::auto_ptr<matrix_return<T>>
      here - using @c typename @c matrix_oper<T>::ptr_type does not
      work 
      @param lhs The left hand operand @f$ A @f$ - the result of a
      previous operation returing a matrix.   
      @param rhs The right hand operand @f$ B @f$ - the result of a
      previous operation returing a matrix.   
      @return new matrix, @f$ C_{ij} = \sum_k A_{ik} B_{kj} @f$ */
  template <typename T>
  typename matrix_oper<T>::ptr_type 
  operator*(std::auto_ptr<matrix_return<T> > lhs, 
	    std::auto_ptr<matrix_return<T> > rhs);
  //  
  /** @} */

  //==================================================================
  /** @{ 
      @name Matrix-vector products @f$ u = A v@f$*/
  /** * operation between a matrix and a vector @f$ u = A v@f$
      The implementation is straight forward.  No hanky-panky here. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_vector_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand, the matrix @f$ A @f$
      @param rhs The right hand operand, the vector @f$ x @f$
      @return new vector, @f$ b_{i} = A_{ij} x_{j} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(const matrix<T>& lhs, const vector<T>& rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector @f$ u = A v@f$
      The implementation is straight forward.  No hanky-panky here. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand, the matrix @f$ op(A) @f$
      @param rhs The right hand operand, the vector @f$ x @f$
      @return new vector, @f$ b_{i} = op(A)_{ij} x_{j} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(const matrix_manip<T> lhs, const vector<T>& rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector  @f$ u = A v@f$
      The implementation is straight forward.  No hanky-panky here.
      This version facilitates calculations like @f$ u = (B C) v@f$
      where @f$ B C @f$ is taken to be the left hand operand @f$
      A@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand, the matrix @f$ A @f$ - the
      result of a previous operation returing a matrix.   
      @param rhs The right hand operand, the vector @f$ x @f$
      @return new vector, @f$ b_{i} = A_{ij} x_{j} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(typename matrix_oper<T>::ptr_type lhs, const vector<T>& rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector  @f$ u = A v@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates calculations like @f$ u = A (B w)@f$
      where @f$ B w @f$ is taken to be the right hand operand @f$
      v@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_vector_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @internal It @e has to be @c std::auto_ptr<vector_return<T>>
      here - using @c typename @c vector_oper<T>::ptr_type does not
      work 
      @param lhs The left hand operand, the matrix @f$ A @f$
      @param rhs The right hand operand, the vector @f$ x @f$  - the
      result of a previous operation returing a vector.   
      @return new vector, @f$ b_{i} = A_{ij} x_{j} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(const matrix<T>& lhs, std::auto_ptr<vector_return<T> > rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector  @f$ u = A v@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates calculations like @f$ u = A (B w)@f$
      where @f$ B w @f$ is taken to be the right hand operand @f$
      v@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @internal It @e has to be @c std::auto_ptr<vector_return<T>>
      here - using @c typename @c vector_oper<T>::ptr_type does not
      work 
      @param lhs The left hand operand, the matrix @f$ op(A) @f$
      @param rhs The right hand operand, the vector @f$ x @f$  - the
      result of a previous operation returing a vector.   
      @return new vector, @f$ b_{i} = op(A)_{ij} x_{j} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(const matrix_manip<T> lhs, std::auto_ptr<vector_return<T> > rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector @f$ u = A v@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates calculations like @f$ u = (B C) (D w)@f$
      where @f$ B C @f$ is taken to be the left hand operand @f$ A@f$,
      and @f$ D w @f$ is taken to be the right hand operand @f$ v@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @internal It @e has to be @c std::auto_ptr<matrix_return<T>>
      here - using @c typename @c matrix_oper<T>::ptr_type does not
      work 
      @internal It @e has to be @c std::auto_ptr<vector_return<T>>
      here - using @c typename @c vector_oper<T>::ptr_type does not
      work 
      @param lhs The left hand operand, the matrix @f$ A @f$ - the
      result of a previous operation returing a matrix.   
      @param rhs The right hand operand, the vector @f$ x @f$ - the
      result of a previous operation returing a vector.   
      @return new vector, @f$ b_{i} = A_{ij} x_{j} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(std::auto_ptr<matrix_return<T> > lhs, 
	    std::auto_ptr<vector_return<T> > rhs);

  /** @} */

  //==================================================================
  /** @{ 
      @name Vector-matrix products @f$ u = v A = A^T v@f$*/
  /** * operation between a matrix and a vector @f$ u = v A = A^T v@f$
      The implementation is straight forward.  No hanky-panky here. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_vector_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand, the vector @f$ v @f$
      @param rhs rhs The right hand operand, the matrix @f$ A @f$
      @return new vector, @f$ b_{i} =  x_{j} A_{ij} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(const vector<T>& lhs, const matrix<T>& rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector @f$ u = v A = A^T v@f$
      The implementation is straight forward.  No hanky-panky here. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand, the vector @f$ v @f$
      @param rhs rhs The right hand operand, the matrix @f$ op(A) @f$
      @return new vector, @f$ b_{i} =  x_{j} op(A)_{ij} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(const vector<T>& lhs, const matrix_manip<T> rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector  @f$ u = v A = A^T v@f$
      The implementation is straight forward.  No hanky-panky here.
      This version facilitates calculations like @f$ u = (B C) v@f$
      where @f$ B C @f$ is taken to be the left hand operand @f$
      A@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand, the vector @f$ v @f$
      @param rhs rhs The right hand operand, the matrix @f$ A @f$  - the
      result of a previous operation returing a matrix.   
      @return new vector, @f$ b_{i} = x_{i} A_{ij} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(const vector<T>& lhs, typename matrix_oper<T>::ptr_type rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector  @f$ u = v A = A^T v@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates calculations like @f$ u = A (B w)@f$
      where @f$ B w @f$ is taken to be the right hand operand @f$
      v@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_vector_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @internal It @e has to be @c std::auto_ptr<vector_return<T>>
      here - using @c typename @c vector_oper<T>::ptr_type does not
      work 
      @param lhs The left hand operand, the vector @f$ v @f$ - the
      result of a previous operation returning a vector.   
      @param rhs rhs The right hand operand, the matrix @f$ A @f$
      @return new vector, @f$ b_{i} =  x_{i} A_{ij} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(std::auto_ptr<vector_return<T> > lhs, const matrix<T>& rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector  @f$ u = v A = A^T v@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates calculations like @f$ u = A (B w)@f$
      where @f$ B w @f$ is taken to be the right hand operand @f$
      v@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @internal It @e has to be @c std::auto_ptr<vector_return<T>>
      here - using @c typename @c vector_oper<T>::ptr_type does not
      work 
      @param lhs The left hand operand, the matrix @f$ op(A) @f$
      @param rhs rhs The right hand operand, the matrix @f$ A @f$  - the
      result of a previous operation returing a vector.   
      @return new vector, @f$ b_{i} = op(A)_{ij} x_{j} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(std::auto_ptr<vector_return<T> > lhs, const matrix_manip<T> rhs);

  //__________________________________________________________________
  /** * operation between a matrix and a vector @f$ u = v A = A^T v@f$
      The implementation is straight forward.  No hanky-panky here. 
      This version facilitates calculations like @f$ u = (B C) (D w)@f$
      where @f$ B C @f$ is taken to be the left hand operand @f$ A@f$,
      and @f$ D w @f$ is taken to be the right hand operand @f$ v@f$. 
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @internal It @e has to be @c std::auto_ptr<matrix_return<T>>
      here - using @c typename @c matrix_oper<T>::ptr_type does not
      work 
      @internal It @e has to be @c std::auto_ptr<vector_return<T>>
      here - using @c typename @c vector_oper<T>::ptr_type does not
      work 
      @param lhs The left hand operand, the vector @f$ v @f$ - the
      result of a previous operation returing a matrix.   
      @param rhs rhs The right hand operand, the matrix @f$ A @f$ - the
      result of a previous operation returing a vector.   
      @return new vector, @f$ b_{i} = A_{ij} x_{j} @f$ */
  template <typename T>
  typename vector_oper<T>::ptr_type 
  operator*(std::auto_ptr<vector_return<T> > lhs, 
	    std::auto_ptr<matrix_return<T> > rhs);

  /** @} */

  //==================================================================
  /** @{ 
      @name Matrix-matrix products */
  /** Compute the matrix-matrix product.  @f$ C = \alpha A B + \beta C@f$
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param alpha The constant @f$ a@f$
      @param a The matrix @f$ A@f$
      @param b The matrix @f$ B@f$
      @param beta The constant @f$ b@f$
      @param c The matrix @f$ C@f$.  On return, this is overwritten by
      the product */
  template <typename T> 
  void 
  matrix_matrix_product(const T& alpha, 
			const matrix<T>& a,
			const matrix<T>& b,
			const T& beta,
			matrix<T>& c);
  //__________________________________________________________________
  /** Compute the matrix-matrix product of manipulated matrices.
      @f$ C = \alpha op_A(A) op_B(B) + \beta C@f$
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param alpha The constant @f$ \alpha @f$
      @param a The manipulator of matrix @f$ op_A(A)@f$
      @param b The manipulator of matrix @f$ op_B(B)@f$
      @param beta The constant @f$ \beta @f$
      @param c The matrix @f$ C@f$.  On return, this is overwritten by
      the product */
  template <typename T> 
  void 
  matrix_matrix_product(const T& alpha, 
			const matrix_manip<T> a,
			const matrix_manip<T> b,
			const T& beta,
			matrix<T>& c);
  //__________________________________________________________________
  /** Compute the matrix-matrix product of manipulated matrices.
      @f$ C = \alpha op_A(A) B + \beta C@f$
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param alpha The constant @f$ \alpha @f$
      @param a The manipulator of matrix @f$ op_A(A)@f$
      @param b The manipulator of matrix @f$ B@f$
      @param beta The constant @f$ \beta @f$
      @param c The matrix @f$ C@f$.  On return, this is overwritten by
      the product */
  template <typename T> 
  void 
  matrix_matrix_product(const T& alpha,
			const matrix_manip<T> a,
			const matrix<T>& b,
			const T& beta,
			matrix<T>& c);
  //__________________________________________________________________
  /** Compute the matrix-matrix product of manipulated matrices.
      @f$ C = \alpha A op_B(B) + \beta C@f$
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_matrix_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param alpha The constant @f$ \alpha @f$
      @param a The manipulator of matrix @f$ A@f$
      @param b The manipulator of matrix @f$ op_B(B)@f$
      @param beta The constant @f$ \beta @f$
      @param c The matrix @f$ C@f$.  On return, this is overwritten by
      the product */
  template <typename T> 
  void 
  matrix_matrix_product(const T& alpha,
			const matrix<T>& a,
			const matrix_manip<T> b,
			const T& beta,
			matrix<T>& c);
  /** @} */
 
  //==================================================================
  /** @{ 
      @name Matrix-vector products */
  /** compute the matrix-vector product and sum 
      @f$ u = \alpha A v + \beta u@f$
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_vector_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param alpha The matrix scalar @f$ \alpha@f$
      @param a The matrix @f$ A@f$
      @param v The vector @f$ v@f$
      @param beta The vector scalar @f$ \beta@f$
      @param u The vector @f$ u@f$, which is overwritten on output by
      the matrix-vector product.  */
  template <typename Type>
  void 
  matrix_vector_product(const Type&         alpha, 
			const matrix<Type>& a, 
			const vector<Type>& v,
			const Type&         beta, 
			vector<Type>&       u);
  /** compute the matrix-vector product and sum 
      @f$ u = \alpha op_A(A) v + \beta u@f$
      If the BLAS headers are included, then the BLAS function
      gslmm::matrix_vector_product will be used for those types it's
      defined for (@c double, @c float, and @c complex<double>). 
      @ingroup matrix
      @param alpha The matrix scalar @f$ \alpha@f$
      @param a The manipulated matrix @f$ op_A(A)@f$
      @param v The vector @f$ v@f$
      @param beta The vector scalar @f$ \beta@f$
      @param u The vector @f$ u@f$, which is overwritten on output by
      the matrix-vector product. */
  template <typename Type>
  void 
  matrix_vector_product(const Type&               alpha, 
			const matrix_manip<Type>  a, 
			const vector<Type>&       v,
			const Type&               beta, 
			vector<Type>&             u);
  /** @} */
}


#ifndef GSLMM_vectormatrix_matrix_manip
# include <gslmm/vectormatrix/matrix_manip.hh>
#endif
#ifndef GSLMM_vectormatrix_matrix_prod
# include <gslmm/vectormatrix/matrix_prod.hh>
#endif
#ifndef GSLMM_vectormatrix_matrix_ret
# include <gslmm/vectormatrix/matrix_ret.hh>
#endif
#ifndef GSLMM_vectormatrix_matrix_oper
# include <gslmm/vectormatrix/matrix_oper.hh>
#endif

#endif
//____________________________________________________________________
//
// EOF
//
