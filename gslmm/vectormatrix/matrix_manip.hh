//
// $Id: matrix_manip.hh,v 1.5 2006-06-05 09:20:09 cholm Exp $ 
//  
//  gslmm::matrix_manip
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
#ifndef GSLMM_vectormatrix_matrix_manip
#define GSLMM_vectormatrix_matrix_manip
#ifndef GSLMM_vectormatrix_matrix_base
# error This header is not for direct inclussion 
#endif
/** @file   matrix_manip.hh
    @author Christian Holm
    @date   Mon Mar 10 14:50:12 2003
    @brief  Basic declarations of matrix manipulators */

#ifndef __GSL_CBLAS_H__
# include <gsl/gsl_cblas.h>
#endif

namespace gslmm
{
  //_________________________________________________________________
  /** @{ 
      @name Matrix manipulator - enumerations */
  /** Types of matrix
      @ingroup matrix
   */
  enum type {
    /** General matrix */
    general_type, 
    /** Triangular matrix, meaning it has the from 
      @f[
        \left(\begin{array}{ccc}
	     x & x & x \\
	       & x & x \\
	       &   & x \\
	   \end{array}\right)
      @f]
    */
    triangular_type, 
    /** Triangular matrix, meaning it has the from 
      @f[
        \left(\begin{array}{ccc}
	     x & x & x \\
	       & x & x \\
	       &   & x \\
	   \end{array}\right)
      @f]
      and we should use the inverse of that. 
    */
    inverse_triangular_type, 
    /** Symmetric matrix, meaning that @f$ x_{ij} = x_{ji}@f$ 
	Note, that only the upper or lower triangle is accessed.  That
	is, only the upper or lower triangle needs to be populated.
	The other part is taken to be the same. */
    symmetric_type, 
    /** Hermitian matrix, meaning that @f$ z_{ij} = \bar{z_{ji}}@f$ 
	Note, that only the upper or lower triangle is accessed.  That
	is, only the upper or lower triangle needs to be populated.
	The other part is taken to be the hermitian conjugate of the
	referenced part. */
    hermitian_type
  };
  //_________________________________________________________________
  /** Operations on matrix passed to the routines. 
      @ingroup matrix
   */
  enum transform {
    /** Do no operation */
    no_transform = CblasNoTrans, 
    /** Transpose the matrix @f$ A = A^T@f$ - that is @f$ A^T_{ij} =
	A_{ji}@f$ */
    transpose_transform  = CblasTrans, 
    /** transpose conjugate the matrix @f$ A = A^H@f$  - that is @f$ A^T_{ij} =
	A_{ji}^\dagger @f$ */
    adjoint_transform    = CblasConjTrans
  };
  //_________________________________________________________________
  /** Storage of triangle matrix.  Chooses where the matrix is stored:
      in the upper or lower triangle of the matrix. 
      @ingroup matrix
   */
  enum location {
    /** Reference the upper triangle of a matrix 
      @f[
        \left(\begin{array}{ccc}
	     x & x & x \\
	       & x & x \\
	       &   & x \\
	   \end{array}\right)
      @f]
    */
    upper_triangle = CblasUpper, 
    /** Reference the lower triangle of a matrix 
      @f[
        \left(\begin{array}{ccc}
	     x &   &   \\
	     x & x &   \\
	     x & x & x \\
	   \end{array}\right)
      @f]
    */
    lower_triangle = CblasLower
  };
  //_________________________________________________________________
  /** Whether it is safe to assume that the diagnal is the unit
      matrix, or not.  
      @ingroup matrix
   */
  enum diagonal {
    /** Do not assume diagonal is unit 
	@f$ \exists i : A_{ii} \neq 1 @f$*/
    non_unit_diagonal = CblasNonUnit,
    /** Assume diagonal is unit (1's) 
	The matrix is assumed to be of the form 
        @f[
        \left(\begin{array}{ccc}
	     1 & & \\
	     & 1 & \\
	     & & \ddots \\
	   \end{array}\right)
        @f] */
    unit_diagonal     = CblasUnit        
  };

  //_________________________________________________________________
  /** Order of the multiplication. This flag is only valid for
	triangular and inverse triangular matrices.
      @ingroup matrix
   */
  enum side {
    /** Do left-hand side multiplication */
    left_side   = CblasLeft,
    /** Do right-hand side multiplication */
    right_side  = CblasRight
  };
  
  //_________________________________________________________________
  /** Order or matrices.  Not really used. 
      @ingroup matrix
   */
  enum order {
    /** The matrix is stored row wise, that is the real index @f$ n@f$
	into a 1-dimensional array of element @f$ A_{ij}@f$ is
	calculated as  
	@f[
	  n = i \cdot N + j 
	@f]
	where @f$ N@f$ is the number of columns, @f$ i@f$ is the
	elements row number, and @f$ j@f$ is the elements column
	number.  This is the C/C++ convention. */
    row_major    = CblasRowMajor,
    /** The matrix is stored column wise, that is the real index @f$ n@f$
	into a 1-dimensional array of element @f$ A_{ij}@f$ is
	calculated as  
	@f[
	  n = i + \cdot M + j 
	@f]
	where @f$ M@f$ is the number of rows, @f$ i@f$ is the
	elements row number, and @f$ j@f$ is the elements column
	number.  This is the Fortran convention. */    
    column_major = CblasColMajor
  };
  /** @} */

  //__________________________________________________________________
  /** @struct matrix_manip matrix/matrix_manip.hh <gslmm/vectormatrix/matrix_base.hh>
      @brief Manipulator of matrixes
      @ingroup matrix
      @param T The type of the matrix. */
  template <typename T> 
  struct matrix_manip
  {
    /** Type of the stored matrix */
    typedef matrix<T> matrix_type;
    /** Type of value */ 
    typedef T value_type;
    /** Type of associated vector */
    typedef vector<T> vector_type;
    /** Type of trait */
    typedef type_trait<T> trait_type;
    /** Type of trait */
    typedef typename trait_type::reference_type reference_type;
    /** Reference to matrix */
    const matrix_type& _matrix;
    
    /** What type the matrix is */
    type      _type;
    /** What transform to apply to the matrix */
    transform _transform;
    /** In triangular, symmetric, an hermitian matricies, where the
	data is stored in the matrix */
    location  _location;
    /** Whether the diagonal is assumed to be of unity */
    diagonal  _diagonal;
    /** What side to use */
    side      _side;
    
    /** Construct a reference manipulator */
    matrix_manip(const     matrix_type& m, 
		 type      k=general_type,
		 transform t=no_transform, 
		 location  l=upper_triangle, 
		 diagonal  d=non_unit_diagonal, 
		 side      s=right_side)
      : _matrix(m), 
	_type(k), 
	_transform(t), 
	_location(l), 
	_diagonal(d),
	_side(s)
    {}
    /** @return Get the row size */
    size_t row_size() const 
    { 
      switch (_transform) {
      case transpose_transform:
      case adjoint_transform:
	return _matrix.column_size(); 
      }
      return _matrix.row_size(); 
    }
    /** @return Get the column size */
    size_t column_size() const 
    { 
      switch (_transform) {
      case transpose_transform:
      case adjoint_transform:
	return _matrix.row_size(); 
      }
      return _matrix.column_size(); 
    }
    /** @param row Row number 
	@param col Column number 
	@return @f$ T(m)_{i,j}@f$ where @f$ T@f$ is the transform. */
    const value_type operator()(size_t row, size_t col) const
    {
      static const value_type& unit = trait_type::unit();
      static const value_type& null = trait_type::null();
      bool   conj = false;
      size_t i    = row, j = col;
      // swap indicies if we are to do a transpose or adjoint operation
      switch (_transform) {
      case transpose_transform:
      case adjoint_transform:  i = col; j = row; break;
      }
      // Now, switch on the type, and manipulate and/or return as needed.
      switch (_type) {
      case inverse_triangular_type: // Should have special case
	GSL_ERROR_VAL("inverse of triangular not implemented", 
		      GSL_EUNIMPL, unit);
	break;
      case triangular_type:
	// Return 1 immediately if the diagonal is requested, and it's 
	// to be considered unit. 
	if (i == j && _diagonal == unit_diagonal) return unit;
	// Check location
	switch (_location) {
	case upper_triangle:
	  // if the column number is strictly greater than the number
	  // of  the row, return 0
	  if (i > j) return null;
	  break;
	case lower_triangle:
	  // if the row number is strictly less than the number of
	  // columns minus 1 minus the column number, return 0. 
	  if (j > i) return null;
	  break;
	}
	break;
      case symmetric_type:
	i = row; j = col;
	switch (_location) {
	case upper_triangle:
	  // if the column number is strictly greater than the number
	  // of  the row, swap the indicies. 
	  if (row > col) { i = col; j = row; }
	  break;
	case lower_triangle:
	  // if the row number is strictly less than the number of
	  // columns minus 1 minus the column number, return 0. 
	  if (col > row) { i = col; j = row; }
	  break;
	}
	break;
      case hermitian_type: // Should have special case
	i = row; j = col;
	switch (_location) {
	case upper_triangle:
	  // if the column number is strictly greater than the number
	  // of  the row, swap the indicies. 
	  if (row > col) { i = col; j = row; conj = true; }
	  break;
	case lower_triangle:
	  // if the row number is strictly less than the number of
	  // columns minus 1 minus the column number, return 0. 
	  if (col > row) { i = col; j = row; conj = true; }
	  break;
	}
	break;
      }
      const value_type& r (conj ?
			   trait_type::conjugate(_matrix(i,j)) : 
			   _matrix(i,j));
      return r;
    }
  };

  
  //==================================================================
  /** @{ 
      @name Matrix manipulator - triangular */ 
  /** Matrix manipulation that says the matrix is triangular. That is,
      it is of the form 
      @f[
       \left(\begin{array}{ccc}
	     x & x & x \\
	       & x & x \\
	       &   & x \\
	   \end{array}\right)
      @f]
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @param d Whether the diagonal can ba assumed to be unity or
      That is, of the form 
      @f[
       \left(\begin{array}{ccc}
	     1 & x & x \\
	       & 1 & x \\
	       &   & 1 \\
	   \end{array}\right)
      @f]
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  triangular(Ref<T> r, location l=upper_triangle, 
	     diagonal d=non_unit_diagonal);

  /** Matrix manipulation that says the matrix is triangular. That is,
      it is of the form 
      @f[
       \left(\begin{array}{ccc}
	     x & x & x \\
	       & x & x \\
	       &   & x \\
	   \end{array}\right)
      @f]
      @ingroup matrix 
      @param m Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @param d Whether the diagonal can ba assumed to be unity or
      That is, of the form 
      @f[
       \left(\begin{array}{ccc}
	     1 & x & x \\
	       & 1 & x \\
	       &   & 1 \\
	   \end{array}\right)
      @f]
      @return A matrix_manip object.  */
  template <typename T>
  inline
  matrix_manip<T>
  triangular(matrix<T>& m, location l=upper_triangle, 
	     diagonal d=non_unit_diagonal) 
  {
    matrix_manip<T> rr(m, triangular_type, no_transform, l, d);
    return rr;
  }

  /** Matrix manipulation that says the matrix is triangular. That is,
      it is of the form 
      @f[
       \left(\begin{array}{ccc}
	     x & x & x \\
	       & x & x \\
	       &   & x \\
	   \end{array}\right)
      @f]
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @param d Whether the diagonal can ba assumed to be unity or
      That is, of the form 
      @f[
       \left(\begin{array}{ccc}
	     1 & x & x \\
	       & 1 & x \\
	       &   & 1 \\
	   \end{array}\right)
      @f]
      @return A matrix_manip object.  */
  template <typename T>
  inline
  matrix_manip<T>&
  triangular(matrix_manip<T> r, location l=upper_triangle, 
	     diagonal d=non_unit_diagonal) 
  {
    r._type     = triangular_type;
    r._location = l;
    r._diagonal = d;
    return r;
  }
  /** @} */

  //____________________________________________________________________
  /** @{ 
      @name Matrix manipulator - triangular, inverse */ 
  /** Matrix manipulation that says the matrix is trianular, and that
      we should use it's inverse. That is, the matrix is of the form 
      @f[
       \left(\begin{array}{ccc}
	     x & x & x \\
	       & x & x \\
	       &   & x \\
	   \end{array}\right)
      @f]
      and we should use the inverse of that. 
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @param d Whether the diagonal can ba assumed to be unity or
      That is, of the form 
      @f[
       \left(\begin{array}{ccc}
	     1 & x & x \\
	       & 1 & x \\
	       &   & 1 \\
	   \end{array}\right)
      @f]
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  inverse_triangular(Ref<T> r, location l=upper_triangle, 
		     diagonal d=non_unit_diagonal);

  /** Matrix manipulation that says the matrix is trianular, and that
      we should use it's inverse. That is, the matrix is of the form 
      @f[
       \left(\begin{array}{ccc}
	     x & x & x \\
	       & x & x \\
	       &   & x \\
	   \end{array}\right)
      @f]
      and we should use the inverse of that. 
      @ingroup matrix 
      @param m Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @param d Whether the diagonal can ba assumed to be unity or
      That is, of the form 
      @f[
       \left(\begin{array}{ccc}
	     1 & x & x \\
	       & 1 & x \\
	       &   & 1 \\
	   \end{array}\right)
      @f]
      @return A matrix_manip object.  */
  template <typename T>
  inline
  matrix_manip<T>
  inverse_triangular(matrix<T>& m, location l=upper_triangle, 
		     diagonal d=non_unit_diagonal) 
  {
    matrix_manip<T> rr(m, inverse_triangular_type, no_transform, l, d);
    return rr;
  }

  /** Matrix manipulation that says the matrix is trianular, and that
      we should use it's inverse. That is, the matrix is of the form 
      @f[
       \left(\begin{array}{ccc}
	     x & x & x \\
	       & x & x \\
	       &   & x \\
	   \end{array}\right)
      @f]
      and we should use the inverse of that. 
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @param d Whether the diagonal can ba assumed to be unity or
      That is, of the form 
      @f[
       \left(\begin{array}{ccc}
	     1 & x & x \\
	       & 1 & x \\
	       &   & 1 \\
	   \end{array}\right)
      @f]
      @return A matrix_manip object.  */
  template <typename T>
  inline
  matrix_manip<T>&
  inverse_triangular(matrix_manip<T> r, location l=upper_triangle, 
		     diagonal d=non_unit_diagonal) 
  {
    r._type     = inverse_triangular_type;
    r._location = l;
    r._diagonal = d;
    return r;
  }
  /** @} */

  //____________________________________________________________________
  /** @{ 
      @name Matrix manipulator - symmetric */
  /** Matrix manipulation that says the matrix is symmetric.  That is,
      @f$ m_{ij} = m_{ji}@f$.
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  symmetric(Ref<T> r, location l=upper_triangle);


  /** Matrix manipulation that says the matrix is symmetric.  That is,
      @f$ m_{ij} = m_{ji}@f$.
      @ingroup matrix 
      @param m Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @return A matrix_manip object.  */
  template <typename T>
  matrix_manip<T> 
  symmetric(matrix<T>& m, location l=upper_triangle)

  {
    matrix_manip<T> rr(m, symmetric_type, no_transform, l);
    return rr;
  }

  /** Matrix manipulation that says the matrix is symmetric.  That is,
      @f$ m_{ij} = m_{ji}@f$.
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @return A matrix_manip object.  */
  template <typename T>
  matrix_manip<T> 
  symmetric(matrix_manip<T> r, location l=upper_triangle)

  {
    r._type     = symmetric_type;
    r._location = l;
    return r;
  }
  /** @} */

  //____________________________________________________________________
  /** @{ 
      @name Matrix manipulator - Hermitian */
  /** Matrix manipulation that says the matrix is hermitian.  That is,
      @f$ m_{ij} = \bar{m_{ij}}@f$.
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  hermitian(Ref<T> r, location l=upper_triangle);


  /** Matrix manipulation that says the matrix is hermitian.  That is,
      @f$ m_{ij} = \bar{m_{ij}}@f$.
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @return A matrix_manip object.  */
  template <typename T>
  matrix_manip<T> 
  hermitian(matrix<T> r, location l=upper_triangle)
  {
    matrix_manip<T> rr(r, hermitian_type, no_transform, l);
    return rr;
  }

  /** Matrix manipulation that says the matrix is hermitian.  That is,
      @f$ m_{ij} = \bar{m_{ij}}@f$.
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @param l Location parameter that says whether the matrix is
      stored in the upper or lower trangle of the object.
      @return A matrix_manip object.  */
  template <typename T>
  matrix_manip<T> 
  hermitian(matrix_manip<T> r, location l=upper_triangle)
  {
    r._type     = hermitian_type;
    r._location = l;
    return r;
  }
  /** @} */

  //____________________________________________________________________
  /** @{ 
      @name Matrix manipulator - transposition */
  /** Matrix manipulation that says we should make a transpose
      operation on the matrix @f$ A = A^T@f$ - that is 
      @f$ A^T_{ij} = A_{ji}@f$ 
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  transpose(Ref<T> r);

  /** Matrix manipulation that says we should make a transpose
      operation on the matrix @f$ A = A^T@f$ - that is 
      @f$ A^T_{ij} = A_{ji}@f$ 
      @ingroup matrix 
      @param m Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T>
  matrix_manip<T>
  transpose(matrix<T>& m) 
  {
    matrix_manip<T> r(m, general_type, transpose_transform);
    return r;
  }

  /** Matrix manipulation that says we should make a transpose
      operation on the matrix @f$ A = A^T@f$ - that is 
      @f$ A^T_{ij} = A_{ji}@f$ 
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T>
  matrix_manip<T>
  transpose(matrix_manip<T> r) 
  {
    r._transform = (r._transform == transpose_transform ? 
		    no_transform : transpose_transform);
    return r;
  }
  /** @} */

  //____________________________________________________________________
  /** @{
      @name Matrix manipulator - adjoint */
  /** Matrix manipulation that says we should make a conjugate
      transpose operation on the matrix @f$ A = A^H@f$  - that is 
      @f$ A^T_{ij} = A_{ji}^\dagger @f$  
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  adjoint(Ref<T> r);

  /** Matrix manipulation that says we should make a conjugate
      transpose operation on the matrix @f$ A = A^H@f$  - that is 
      @f$ A^T_{ij} = A_{ji}^\dagger @f$  
      @ingroup matrix 
      @param m Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  adjoint(matrix<T> m) 
  {
    matrix_manip<T> r(m, general_type, adjoint_transform);
    return r;
  }

  /** Matrix manipulation that says we should make a conjugate
      transpose operation on the matrix @f$ A = A^H@f$  - that is 
      @f$ A^T_{ij} = A_{ji}^\dagger @f$  
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  adjoint(matrix_manip<T> r) 
  {
    r._transform = (r._transform == adjoint_transform ? 
		    no_transform : adjoint_transform);
    return r;
  }
  /** @}  */
    
  //____________________________________________________________________
  /** @{ 
      @name Matrix manipulator - left-multiplication */
  /** Matrix manipulation that says we should do left hand side
      multiplication. 
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  left(Ref<T> r);

  /** Matrix manipulation that says we should do left hand side
      multiplication. 
      @ingroup matrix 
      @param m Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  left(matrix<T> m) 
  {
    matrix_manip<T> r(m, general_type, no_transform, 
		      upper_triangle, non_unit_diagonal, left_side);
    return r;
  }

  /** Matrix manipulation that says we should do left hand side
      multiplication. 
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  left(matrix_manip<T> r) 
  {
    r._side = left_side;
    return r;
  }
  /** @}  */

  //____________________________________________________________________
  /** @{ 
      @name Matrix manipulator - right-multiplication */
  /** Matrix manipulation that says we should do right hand side
      multiplication. 
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  right(Ref<T> r);

  /** Matrix manipulation that says we should do right hand side
      multiplication. 
      @ingroup matrix 
      @param m Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  right(matrix<T> m) 
  {
    matrix_manip<T> r(m, general_type, no_transform, 
		      upper_triangle, non_unit_diagonal, right_side);
    return r;
  }

  /** Matrix manipulation that says we should do right hand side
      multiplication. 
      @ingroup matrix 
      @param r Reference or Matrix object. 
      @return A matrix_manip object.  */
  template <typename T, template <class> class Ref>
  matrix_manip<T> 
  right(matrix_manip<T> r) 
  {
    r._side = right_side;
    return r;
  }
  /** @} */


  //__________________________________________________________________
  /** @ */
  /** @name Output streamers of matrix manipulator */
  template <typename T>
  inline std::ostream& 
  operator<<(std::ostream& o, const matrix_manip<T>& m) 
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
