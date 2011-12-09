//
// $Id: matrix.hh,v 1.2 2006-05-09 07:40:34 cholm Exp $ 
//  
//  gslmm::matrix
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
#ifndef GSLMM_matrix
#define GSLMM_matrix

/** @file   matrix.hh
    @author Christian Holm
    @date   Mon Sep 16 04:46:14 2002
    @brief  Declaration of matrix classses. 
    A base template class as well as some specialisation are declared
    in this file. */

/** @defgroup matrix Matrices
    @ingroup vectormatrix

    This group contains matrix classes, and classes and functions to
    manipulate matrices.  

    The class matrix is the @b GSL-- representation of a matrix.  It
    is implemented (as it is in @b GSL) as a dense matrix.  That is,
    room for all elements is allocated. 

    A matrix has various a few properties 
    -  Number of rows (gslmm::matrix<T>::row_size())
    -  Number of columns (gslmm::matrix<T>:columns_size())
    -  @f$ M \times N@f$ elements, where @f$ M@f$ is the number of
       rows, and @f$ N@f$ is the number of columns. 

    Vadrious operations are defined on matrices, including sub-views,
    retrivial of the diagonal, and super- and sub-diagonals. 

    Normal matrix-matrix, matrix-vector, and matrix-scalar
    operations are defined as well.  If the matrices @f$ A, A' \in
    S_{M\times N}@f$, @f$ B \in S_{N\times P}@f$, @f$ C \in S_{M\times
    P}@f$, and the vectors @f$ v\in S_{M}@f$, @f$ u\in S_{N}@f$, and
    the scalar @f$ a \in S@f$ for some set of numbers @f$ S =
    \mathbb{R}, \mathbb{C}, \mathbb{Z}@f$, then the following
    operations are defined  
    @f{align*}
    A' &= A a  \\
    A' &= A + a \\
    u  &= A v   \\ 
    C  &= A B
    @f}
    
    These operations are implemented via the corresponding @e global
    C++ operators ( 
    operator*(const matrix<T>&,const double&),
    operator+(const matrix<T>&,const double&),
    operator*(const matrix<T>&,const vector<T>&),
    operator*(const matrix<T>&,const matrix<T>&)).  For the
    matrix-vector and matrix-matrix product, the operators call the
    global functions matrix_vector_product and matrix_matrix_product
    respectively.  

    Note, that the global operators for @f$ A v@f$ and @f$ A B@f$ as
    well as their corresponding functions, do not return a new vector
    or matrix.  Instead the return an object of a specialised class
    (_matrix_vector_mult for @f$ A v@f$, and _matrix_matrix_mult for
    @f$ A B@f$).   This is to allow for defered evaluation of the
    potentially expensive operation of @f$ A v@f$ or @f$ A B@f$.
    (This follows the outline of section 22.4.7 of @e The @e C++ 
    @e Programming @e Language (Special Edition) by Bjarne
    Stroustrup). 

    The special object simple contains references to the two operands
    of the operation.  When a vector or matrix is assigned from this
    object, the operation is evaluated in-place - that is,  without a
    temporary object.  For example 

    @code 
    size_t m=3, n=2, p=4;           // 1: Dimension
    gslmm::matrix<double> A(m,n);   // 2: Left-hand operand
    gslmm::matrix<double> B(n,p);   // 3: Right-hand operand
    gslmm::matrix<double> C(m,p)    // 4: To be assigned A * B
    fill_matrix(A);                 // 5: Set elements of A
    fill_matrix(B);                 // 6: Set elements of B
    C = A * B;                      // 7: Assign A * B to C
    @endcode
    
    If the @c * operator was defined to return a new matrix, then in
    line 7 we could have potentially 2 useless temporaries: 
    - First, we would make a temporary @f$ T@f$ in the @c operator*
      function, and assign the elements  @f$ T_{ij} = \sum_{k} A_{ik}
      B_{kj} @f$.   This would be one un-wanted construction.
    - Since we could not return a reference to this object, as it goes
      out of scope when leaving the @c operator* function, we'd have
      to copy @f$ T@f$ onto the return stack as @f$ T'@f$.   That is,
      we'd do a copy construction for the return object @f$ T'@f$, and
      we'd destruct @f$ T@f$. 
    - Finally, we would assign to @f$ C@f$ the elements of @f$ T'@f$,
      and then destroy @f$ T'@f$ 

    That is, we'd have one un-wanted construction, one un-wanted copy
    construction, and to un-wanted destructions.  

    However, in the implementation used here, we construct a small
    special object in the @c operator* function, and copy that back on
    the return stack.   Since the special object is small (two
    references to the operands), this is not a large overhead, and
    never depends on the size of the matrices and vectors involved.
    When the assignment operator is envoked in line 7 above, it takes
    care of evaluating the operation, and assign directly to the
    elements of @f$ C@f$. 

    Note, that in the default definition of these operators, no tricks
    has been employed to speed up the calculation.   To use better
    code for these operations, one should include the relevant 
    @ref blas headers.  This makes even more sense when manipulators
    are used to flag properties of a matrix. 

    Matrix manipulators are functions that flags a matrix as having a
    certain property.  For example 
    @code 
    gslmm::matrix<double> A(3,3);  // Matrix
    assign_matrix(A);              // Make a symmetric matrix
    gslmm::symmetric(A);           // Say that A is symmetric 
    @endcode 
    This would tell the system, that @f$ A@f$ is a symmetric matrix,
    @f[
    A = \left(\begin{array}{ccc}
          A_{00} & A_{01} & A_{02} \\
	  A_{01} & A_{11} & A_{12} \\
	  A_{02} & A_{12} & A_{22} \\
        \end{array}\right)
    @f]
    and that we should only reference the upper-right triangle of the
    matrix (the lower-left is the same as the upper-right).  

    The matrix manipulator functions return an object of
    matrix_manip<T>.  The various operators are also definied for
    matrix manipulators, so one can do 
    @code 
    matrix<double> A(...);
    matrix<double> B(...);
    matrix<double> C(...);
    C = gslmm::symmetrix(A) * B;
    @endcode 
    One can easily nest manipulators, if needed, like 
    @code
    matrix<complex<double> > A(...);
    matrix<complex<double> > B(...);
    matrix<complex<double> > C(...);
    C = gslmm::transpose(gslmm::hermitian(A)) * B;
    @endcode 

    Whenever optimised BLAS algorithms exists, and the user has
    included the relevant BLAS headers, they are used. 

    Note, that operators @c operator+=, @c operator-=, @c operator*=, 
    and @c operator/= are @e element-wise operations.  If you think
    about it, you will see why.  For example, suppose @f$ A \in
    S_{M\times N}@f$, @f$ B \in S_{N\times P}@f$, @f$ C \in S_{M\times
    P}@f$, and @f$ N \neq P@f$, then if the @c operator*= function
    implemented @f$ A B@f$ the code 
    @code 
    A *= B;
    @endcode 
    would have to change the dimensions of @f$ A@f$ to @f$ N\times
    P@f$. Changing the dimension of a matrix or vector is now allowed
    in @b GSL-- (nor @b GSL) because it can have very bad and
    unexpected side-effects. 
*/

#ifndef GSLMM_matrix_double
#include <gslmm/vectormatrix/matrix_double.hh>
#endif
#ifndef GSLMM_matrix_long_double
#include <gslmm/vectormatrix/matrix_long_double.hh>
#endif
#ifndef GSLMM_matrix_float
#include <gslmm/vectormatrix/matrix_float.hh>
#endif
#ifndef GSLMM_matrix_unsigned_long
#include <gslmm/vectormatrix/matrix_unsigned_long.hh>
#endif
#ifndef GSLMM_matrix_long
#include <gslmm/vectormatrix/matrix_long.hh>
#endif
#ifndef GSLMM_matrix_unsigned_int
#include <gslmm/vectormatrix/matrix_unsigned_int.hh>
#endif
#ifndef GSLMM_matrix_int
#include <gslmm/vectormatrix/matrix_int.hh>
#endif
#ifndef GSLMM_matrix_unsigned_short
#include <gslmm/vectormatrix/matrix_unsigned_short.hh>
#endif
#ifndef GSLMM_matrix_short
#include <gslmm/vectormatrix/matrix_short.hh>
#endif
#ifndef GSLMM_matrix_unsigned_char
#include <gslmm/vectormatrix/matrix_unsigned_char.hh>
#endif
#ifndef GSLMM_matrix_char
#include <gslmm/vectormatrix/matrix_char.hh>
#endif

#endif
//____________________________________________________________________
//
// EOF
//
