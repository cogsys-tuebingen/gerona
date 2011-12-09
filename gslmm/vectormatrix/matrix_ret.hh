//
// $Id: matrix_ret.hh,v 1.3 2007-04-21 09:03:14 cholm Exp $ 
//  
//  gslmm::matrix_ret
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
#ifndef GSLMM_vectormatrix_matrix_ret
#define GSLMM_vectormatrix_matrix_ret
#ifndef GSLMM_vectormatrix_matrix_base
# error This header is not for direct inclussion 
#endif
/** @file   vectormatrix/matrix_ret.hh
    @author Christian Holm
    @date   Mon Mar 10 14:50:12 2003
    @brief  Base templates for return of matrices classes */
#ifndef GSL_errno
# include <gsl/gsl_errno.h>
#endif

namespace gslmm
{
  //================================================================
  template <typename T, template <class> class Lhs, template <class> class Rhs>
  struct mm_mult_helper 
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, Lhs>            lhs_trait;
    typedef m_arg_trait<T, Rhs>            rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::doit_type   lhs_type;
    typedef typename rhs_trait::doit_type   rhs_type;
    /** @} */
    /** Matrix type */
    typedef matrix<T> matrix_type;
    /** Do the multiplication */
    static void 
    mult(const T& u, lhs_type l, rhs_type r, const T& n, matrix_type& c) 
    {
      matrix_matrix_product(u,l,r,n,c);
    }
  };

  //________________________________________________________________
  template <typename T, template <class> class Lhs>
  struct mm_mult_helper<T, Lhs, matrix_return>
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, Lhs>            lhs_trait;
    typedef m_arg_trait<T, matrix_return>  rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::doit_type   lhs_type;
    typedef typename rhs_trait::doit_type   rhs_type;
    /** @} */
    /** Matrix type */
    typedef matrix<T> matrix_type;
    /** Do the multiplication */
    static void 
    mult(const T& u, lhs_type l, rhs_type r, const T& n, matrix_type& c) 
    {
      matrix<T> t(rhs_trait::row_size(r),rhs_trait::column_size(r));
      rhs_trait::apply(r, t);
      matrix_matrix_product(u,l,t,n,c);
    }
  };
  //________________________________________________________________
  template <typename T, template <class> class Rhs>
  struct mm_mult_helper<T, matrix_return, Rhs>
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, matrix_return>  lhs_trait;
    typedef m_arg_trait<T, Rhs>            rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::doit_type lhs_type;
    typedef typename rhs_trait::doit_type rhs_type;
    /** @} */
    /** Matrix type */
    typedef matrix<T> matrix_type;
    /** Do the multiplication */
    static void 
    mult(const T& u, lhs_type l, rhs_type r, const T& n, matrix_type& c) 
    {
      matrix<T> t(lhs_trait::row_size(l),lhs_trait::column_size(l));
      lhs_trait::apply(l, t);
      matrix_matrix_product(u,t,r,n,c);
    }
  };
  //________________________________________________________________
  template <typename T>
  struct mm_mult_helper<T, matrix_return, matrix_return>
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, matrix_return>  lhs_trait;
    typedef m_arg_trait<T, matrix_return>  rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::doit_type lhs_type;
    typedef typename rhs_trait::doit_type rhs_type;
    /** @} */
    /** Matrix type */
    typedef matrix<T> matrix_type;
    /** Do the multiplication */
    static void 
    mult(const T& u, lhs_type l, rhs_type r, const T& n, matrix_type& c) 
    {
      matrix<T> tl(lhs_trait::row_size(l),lhs_trait::column_size(l));
      matrix<T> tr(rhs_trait::row_size(r),rhs_trait::column_size(r));
      lhs_trait::apply(l, tl);
      rhs_trait::apply(r, tr);
      matrix_matrix_product(u,tl,tr,n,c);
    }
  };
    
  //================================================================
  /** @brief Class that represents a matrix-matrix product 
      @ingroup matrix 
  */
  template <typename T, template <class> class Lhs, template <class> class Rhs>
  struct mm_mult : public matrix_return<T>
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, Lhs>            lhs_trait;
    typedef m_arg_trait<T, Rhs>            rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::ctor_type   lhs_ctype;
    typedef typename rhs_trait::ctor_type   rhs_ctype;
    /** @} */
    /** @{ 
	@name type of members */
    typedef typename lhs_trait::member_type lhs_mtype;
    typedef typename rhs_trait::member_type rhs_mtype;
    /** @} */
    /** @{ 
	@name type of @c operator* arguments */
    typedef typename lhs_trait::oper_type   lhs_otype;
    typedef typename rhs_trait::oper_type   rhs_otype;
    /** @} */
    /** @{ 
	@name type of @c doit arguments */
    typedef typename lhs_trait::doit_type   lhs_dtype;
    typedef typename rhs_trait::doit_type   rhs_dtype;
    /** @} */
    /** Matrix type */
    typedef matrix<T>                       matrix_type;
    /** Type of return value from operator */
    typedef matrix_return<T>                return_type;
    /** Type of return value from operator */
    typedef typename matrix_oper<T>::ptr_type ptr_type;
    /** Type of trait of element type */
    typedef type_trait<T>                   trait_type;
    /** @c operator* is a friend */
    friend typename matrix_oper<T>::ptr_type operator*<>(lhs_otype,rhs_otype);
    /** @return Row size of product */
    size_t row_size() const { return lhs_trait::row_size(_lhs); }
    /** @return Column size of product */
    size_t column_size() const { return rhs_trait::column_size(_rhs); }
    void operator()(matrix_type& c) const
    {
      const T& u = trait_type::unit();
      const T& n = trait_type::null();
      mm_mult_helper<T,Lhs,Rhs>::mult(u, _lhs, _rhs, n, c);
    }
  protected:
    mm_mult(lhs_ctype lhs, rhs_ctype rhs) 
      : _lhs(lhs), _rhs(rhs) 
    {
      lhs_trait::check(_lhs);
      rhs_trait::check(_rhs);
    }
    /** Left hand side operand */
    lhs_mtype _lhs;
    /** Right hand side operand */
    rhs_mtype _rhs;
  };

  //================================================================
  template <typename T, template <class> class Lhs, template <class> class Rhs>
  struct mv_mult_helper 
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, Lhs> lhs_trait;
    typedef v_arg_trait<T, Rhs> rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::doit_type   lhs_type;
    typedef typename rhs_trait::doit_type   rhs_type;
    /** @} */
    /** Matrix type */
    typedef matrix<T>                       matrix_type;
    /** Vector type */
    typedef vector<T>                       vector_type;
    static void 
    mult(const T& u, lhs_type l, rhs_type r, const T& n, vector_type& c) 
    {
      matrix_vector_product(u,l,r,n,c);
    }
  };

  //________________________________________________________________
  template <typename T, template <class> class Lhs>
  struct mv_mult_helper<T, Lhs, vector_return>
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, Lhs>            lhs_trait;
    typedef v_arg_trait<T, vector_return>  rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::doit_type   lhs_type;
    typedef typename rhs_trait::doit_type   rhs_type;
    /** @} */
    /** Matrix type */
    typedef matrix<T> matrix_type;
    /** Vector type */
    typedef vector<T> vector_type;
    static void 
    mult(const T& u, lhs_type l, rhs_type r, const T& n, vector_type& c) 
    {
      vector<T> t(rhs_trait::size(r));
      rhs_trait::apply(r, t);
      matrix_vector_product(u,l,t,n,c);
    }
  };
  //________________________________________________________________
  template <typename T, template <class> class Rhs>
  struct mv_mult_helper<T, matrix_return, Rhs>
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, matrix_return>  lhs_trait;
    typedef v_arg_trait<T, Rhs>            rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::doit_type   lhs_type;
    typedef typename rhs_trait::doit_type   rhs_type;
    /** @} */
    /** Matrix type */
    typedef matrix<T> matrix_type;
    /** Vector type */
    typedef vector<T> vector_type;
    static void 
    mult(const T& u, lhs_type l, rhs_type r, const T& n, vector_type& c) 
    {
      matrix<T> t(lhs_trait::row_size(l),lhs_trait::column_size(l));
      lhs_trait::apply(l, t);
      matrix_vector_product(u,t,r,n,c);
    }
  };
  //________________________________________________________________
  template <typename T>
  struct mv_mult_helper<T, matrix_return, vector_return>
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, matrix_return>  lhs_trait;
    typedef v_arg_trait<T, vector_return>  rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::doit_type  lhs_type;
    typedef typename rhs_trait::doit_type  rhs_type;
    /** @} */
    /** Matrix type */
    typedef matrix<T>  matrix_type;
    /** Vector type */
    typedef vector<T>  vector_type;
    static void 
    mult(const T& u, lhs_type l, rhs_type r, const T& n, vector_type& c) 
    {
      matrix<T> tl(lhs_trait::row_size(l),lhs_trait::column_size(l));
      vector<T> tr(rhs_trait::size(r));
      lhs_trait::apply(l, tl);
      rhs_trait::apply(r, tr);
      matrix_vector_product(u,tl,tr,n,c);
    }
  };

  //================================================================
  /** @brief Class that represents a matrix-vector product 
      @ingroup matrix 
  */
  template <typename T, template <class> class Lhs, template <class> class Rhs>
  struct mv_mult : public vector_return<T>
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, Lhs>            lhs_trait;
    typedef v_arg_trait<T, Rhs>            rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::ctor_type   lhs_ctype;
    typedef typename rhs_trait::ctor_type   rhs_ctype;
    /** @} */
    /** @{ 
	@name type of members */
    typedef typename lhs_trait::member_type lhs_mtype;
    typedef typename rhs_trait::member_type rhs_mtype;
    /** @} */
    /** @{ 
	@name type of @c operator* arguments */
    typedef typename lhs_trait::oper_type   lhs_otype;
    typedef typename rhs_trait::oper_type   rhs_otype;
    /** @} */
    /** @{ 
	@name type of @c doit arguments */
    typedef typename lhs_trait::doit_type   lhs_dtype;
    typedef typename rhs_trait::doit_type   rhs_dtype;
    /** @} */
    /** Matrix type */
    typedef matrix<T>                       matrix_type;
    /** Matrix type */
    typedef vector<T>                       vector_type;
    /** Type of return value from operator */
    typedef vector_return<T>                return_type;
    /** Type of return value from operator */
    typedef std::auto_ptr<return_type>      oper_type;
    /** Type of trait of element type */
    typedef type_trait<T>                   trait_type;
    /** @c operator* is a friend */
    friend oper_type operator*<>(lhs_otype,rhs_otype);
    /** @return Row size of product */
    size_t size() const { return lhs_trait::row_size(_lhs); }
    void operator()(vector_type& c) const
    {
      const T& u = trait_type::unit();
      const T& n = trait_type::null();
      mv_mult_helper<T,Lhs,Rhs>::mult(u, _lhs, _rhs, n, c);
    }
  protected:
    mv_mult(lhs_ctype lhs, rhs_ctype rhs) 
      : _lhs(lhs), _rhs(rhs) 
    {
      lhs_trait::check(_lhs);
      rhs_trait::check(_rhs);
    }
    /** Left hand side operand */
    lhs_mtype _lhs;
    /** Right hand side operand */
    rhs_mtype _rhs;
  };

  //================================================================
  /** @brief Class that represents a matrix-vector product 
      @ingroup matrix 
  */
  template <typename T, template <class> class Lhs, template <class> class Rhs>
  struct vm_mult : public vector_return<T>
  {
    /** @{ 
	@name type of traits */
    typedef m_arg_trait<T, Lhs>            lhs_trait;
    typedef v_arg_trait<T, Rhs>            rhs_trait;
    /** @} */
    /** @{ 
	@name type of constructor arguments */
    typedef typename lhs_trait::ctor_type   lhs_ctype;
    typedef typename rhs_trait::ctor_type   rhs_ctype;
    /** @} */
    /** @{ 
	@name type of members */
    typedef typename lhs_trait::member_type lhs_mtype;
    typedef typename rhs_trait::member_type rhs_mtype;
    /** @} */
    /** @{ 
	@name type of @c operator* arguments */
    typedef typename lhs_trait::oper_type   lhs_otype;
    typedef typename rhs_trait::oper_type   rhs_otype;
    /** @} */
    /** @{ 
	@name type of @c doit arguments */
    typedef typename lhs_trait::doit_type   lhs_dtype;
    typedef typename rhs_trait::doit_type   rhs_dtype;
    /** @} */
    /** Matrix type */
    typedef matrix<T>                       matrix_type;
    /** Matrix type */
    typedef vector<T>                       vector_type;
    /** Type of return value from operator */
    typedef vector_return<T>                return_type;
    /** Type of return value from operator */
    typedef std::auto_ptr<return_type>      oper_type;
    /** Type of trait of element type */
    typedef type_trait<T>                   trait_type;
    /** @c operator* is a friend */
    friend oper_type operator*<>(lhs_otype,rhs_otype);
    /** @return Row size of product */
    size_t size() const { return lhs_trait::row_size(_lhs); }
    void operator()(vector_type& c) const
    {
      const T& u = trait_type::unit();
      const T& n = trait_type::null();
      mv_mult_helper<T,Lhs,Rhs>::mult(u, tranpose(_lhs), _rhs, n, c);
    }
  protected:
    vm_mult(lhs_ctype lhs, rhs_ctype rhs) 
      : _lhs(lhs), _rhs(rhs) 
    {
      lhs_trait::check(_lhs);
      rhs_trait::check(_rhs);
    }
    /** Left hand side operand */
    lhs_mtype _lhs;
    /** Right hand side operand */
    rhs_mtype _rhs;
  };
}
    
#endif
//____________________________________________________________________
//
// EOF
//
