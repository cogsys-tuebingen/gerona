//
// $Id: matrix_oper.hh,v 1.5 2007-04-21 09:03:14 cholm Exp $ 
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
#ifndef GSLMM_vectormatrix_matrix_oper
#define GSLMM_vectormatrix_matrix_oper
#ifndef GSLMM_vectormatrix_matrix_base
# error This header is not for direct inclussion 
#endif
/** @file   vectormatrix/matrix_oper.hh
    @author Christian Holm
    @date   Mon Mar 10 14:50:12 2003
    @brief  Base template for matricies classes */
#ifndef GSL_errno
# include <gsl/gsl_errno.h>
#endif

namespace gslmm
{
  //__________________________________________________________________
  /** @{ */
  /** @name General operations on matricies */
  /** + operation between matricies (element by element)
      @ingroup matrix
      @param lhs The left hand operand @f$ l @f$
      @param rhs The right hand operand @f$ r @f$
      @return new matrix, @f$ m_{ij} = l_{ij} + r_{ij} @f$ */
  template <typename T>
  inline matrix<T>
  operator+(const matrix<T>& lhs, const matrix<T>& rhs)
  {
    matrix<T> r(lhs); 
    r += rhs;
    return r;
  }
  //__________________________________________________________________
  /** - operation between matricies (element by element)
      @ingroup matrix
      @param lhs The left hand operand @f$ l @f$
      @param rhs The right hand operand @f$ r @f$
      @return new matrix, @f$ m_{ij} = l_{ij} - r_{ij} @f$ */
  template <typename T>
  inline matrix<T>
  operator-(const matrix<T>& lhs, const matrix<T>& rhs)
  {
    matrix<T> r(lhs); 
    r -= rhs;
    return r;
  }
  //__________________________________________________________________
  /** / operation between matricies 
      This is generally not defined.  It will simply return a copy of
      @a lhs (division by identity :-)
      If the linear algebra header (gslmm/linear/linear.hh) is
      included, then this is implemented using decompositions for a
      subset of the available types (all sorts of matrices of @c
      double, square matrices of @c complex<double>). 
      @ingroup matrix
      @param lhs The left hand operand @f$ l @f$
      @param rhs The right hand operand @f$ r @f$
      @return new matrix, @f$ m_{ij} = l_{ij} / r_{ij} @f$ */
  template <typename T>
  inline matrix<T>
  operator/(const matrix<T>& lhs, const matrix<T>& rhs)
  {
    matrix<T> r(lhs); 
    return r;
  }
  /** @} */

  //__________________________________________________________________
  /** @{ */
  /** @name General operations on matrix-scaler */
  /** + operation between matrix and number (element by element)
      @ingroup matrix
      @param a The number.
      @param m The matrix.
      @return new matrix, @f$ r_{ij} = m_{ij} + a@f$ */
  template <typename T>
  inline matrix<T>
  operator+(const matrix<T>& m, const double& a)
  {
    matrix<T> r(m); 
    r += a;
    return r;
  }
  //__________________________________________________________________
  /** - operation between matrix and number (element by element)
      @ingroup matrix
      @param a The number.
      @param m The matrix.
      @return new matrix, @f$ r_{ij} = m_{ij} - a @f$ */
  template <typename T>
  inline matrix<T>
  operator-(const matrix<T>& m, const double& a)
  {
    matrix<T> r(m); 
    r -= a;
    return r;
  }
  //__________________________________________________________________
  /** * operation between matrix and number (element by element)
      @ingroup matrix
      @param a The number.
      @param m The matrix.
      @return new matrix, @f$ r_{ij} = a * m_{ij} @f$ */
  template <typename T>
  inline matrix<T>
  operator*(const matrix<T>& m, const double& a)
  {
    matrix<T> r(m); 
    r *= a;
    return r;
  }
  //__________________________________________________________________
  /** / operation between matrix and number (element by element)
      @ingroup matrix
      @param a The number.
      @param m The matrix.
      @return new matrix, @f$ r_{ij} = m_{ij} / a@f$ */
  template <typename T>
  inline matrix<T>
  operator/(const matrix<T>& m, const double& a)
  {
    matrix<T> r(m); 
    r /= a;
    return r;
  }
  /** @} */

  //==================================================================
  template <typename T>
  inline typename matrix_oper<T>::ptr_type
  operator*(const matrix<T>& lhs, const matrix<T>& rhs)
  {
    typedef mm_mult<T,matrix,matrix> return_type;
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs,rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename matrix_oper<T>::ptr_type
  operator*(const matrix<T>& lhs, const matrix_manip<T> rhs)
  {
    typedef mm_mult<T,matrix,matrix_manip> return_type;
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs,rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename matrix_oper<T>::ptr_type
  operator*(const matrix<T>& lhs, std::auto_ptr<matrix_return<T> > rhs)
    // typename matrix_oper<T>::ptr_type rhs)
  {
    typedef mm_mult<T,matrix,matrix_return> return_type;
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs,rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename matrix_oper<T>::ptr_type
  operator*(const matrix_manip<T> lhs, const matrix<T>& rhs)
  {
    typedef mm_mult<T,matrix_manip,matrix> return_type;
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs,rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename matrix_oper<T>::ptr_type
  operator*(const matrix_manip<T> lhs, const matrix_manip<T> rhs)
  {
    typedef mm_mult<T,matrix_manip,matrix_manip> return_type;
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs,rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename matrix_oper<T>::ptr_type
  operator*(const matrix_manip<T> lhs, std::auto_ptr<matrix_return<T> > rhs)
  {
    typedef mm_mult<T,matrix_manip,matrix_return> return_type;
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs,rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }

  //__________________________________________________________________
  template <typename T>
  inline typename matrix_oper<T>::ptr_type
  operator*(typename matrix_oper<T>::ptr_type lhs, const matrix<T>& rhs)
  {
    typedef mm_mult<T,matrix_return,matrix> return_type;
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs,rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename matrix_oper<T>::ptr_type
  operator*(typename matrix_oper<T>::ptr_type lhs, const matrix_manip<T> rhs)
  {
    typedef mm_mult<T,matrix_return,matrix_manip> return_type;
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs,rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename matrix_oper<T>::ptr_type
  operator*(std::auto_ptr<matrix_return<T> > lhs,
	    std::auto_ptr<matrix_return<T> > rhs)
  {
    typedef mm_mult<T,matrix_return,matrix_return> return_type;
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs,rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }

  //==================================================================
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(const matrix<T>& lhs, const vector<T>& rhs)
  {
    typedef mv_mult<T,matrix,vector> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(const matrix_manip<T> lhs, const vector<T>& rhs)
  {
    typedef mv_mult<T,matrix_manip,vector> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(typename matrix_oper<T>::ptr_type lhs, 
	    const vector<T>& rhs)
  {
    typedef mv_mult<T,matrix_return,vector> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(const matrix<T>& lhs, std::auto_ptr<vector_return<T> > rhs)
  {
    typedef mv_mult<T,matrix,vector_return> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(const matrix_manip<T> lhs, std::auto_ptr<vector_return<T> > rhs)
  {
    typedef mv_mult<T,matrix_manip,vector_return> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
#if 1
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(std::auto_ptr<matrix_return<T> > lhs, 
	    std::auto_ptr<vector_return<T> > rhs)
  {
    typedef mv_mult<T,matrix_return,vector_return> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
#endif

  //==================================================================
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(const vector<T>& lhs, const matrix<T>& rhs)
  {
    typedef vm_mult<T,matrix,vector> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(const vector<T>& lhs, const matrix_manip<T> rhs)
  {
    typedef vm_mult<T,matrix_manip,vector> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(const vector<T>& lhs, typename matrix_oper<T>::ptr_type rhs)
  {
    typedef vm_mult<T,matrix_return,vector> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(std::auto_ptr<vector_return<T> > lhs, const matrix<T>& rhs)
  {
    typedef vm_mult<T,matrix,vector_return> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(std::auto_ptr<vector_return<T> > lhs, const matrix_manip<T> rhs)
  {
    typedef vm_mult<T,matrix_manip,vector_return> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
#if 1
  //__________________________________________________________________
  template <typename T>
  inline typename vector_oper<T>::ptr_type
  operator*(std::auto_ptr<vector_return<T> > lhs, 
	    std::auto_ptr<matrix_return<T> > rhs)
  {
    typedef vm_mult<T,matrix_return,vector_return> return_type;
    typedef typename vector_oper<T>::ptr_type ptr_type;
    return_type* r = new return_type(lhs, rhs);
    // std::cout << "Return " << r << std::endl;
    return ptr_type(r);
  }
#endif

  //==================================================================
  /** @ */
  /** @name Output streamers of matrix */
  template <typename T>
  inline std::ostream& 
  operator<<(std::ostream& o, const matrix<T>& m) 
  {
    for (size_t r = 0; r < m.row_size(); r++) {
      for (size_t c = 0; c < m.column_size(); c++) 
	o << m(r,c) << '\t';
      if (r + 1 != m.row_size()) o << '\n';
    }
    return o;
  }
  /** @} */
}

#endif
//____________________________________________________________________
//
// EOF
//
