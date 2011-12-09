//
// $Id: math-test.cc,v 1.8 2006-05-01 14:24:57 cholm Exp $ 
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
#ifndef GSL_utility
#include <gslmm/math/utility.hh>
#endif
#ifndef GSL_constant
#include <gslmm/math/constant.hh>
#endif
#ifndef GSL_math_type_trait
#include <gslmm/math/type_trait.hh>
#endif
#ifndef GSL_machine
#include <gslmm/math/machine.hh>
#endif
#ifndef GSL_test_suite
#include <gslmm/test/test_suite.hh>
#endif
#ifndef __IOSTREAM__
#include <iostream>
#endif

/** @file   math/math-test.cc
    @author Christian Holm
    @date   Thu Mar 06 03:03:35 2003
    @brief  Maths test */


int main(int argc, char** argv) 
{
  gslmm::test_suite ts("math", argc, argv);

  double y, y_expected;
  int e, e_expected;

  //___________________________________________________________________
  // Test for expm1 
  y = gslmm::expm1(0.0);
  y_expected = 0.0;
  ts.relative(y, y_expected, 1e-15, "gslmm::expm1(0.0)");

  y = gslmm::expm1(1e-10);
  y_expected = 1.000000000050000000002e-10;
  ts.relative(y, y_expected, 1e-15, "gslmm::expm1(1e-10)");

  y = gslmm::expm1(-1e-10);
  y_expected = -9.999999999500000000017e-11;
  ts.relative(y, y_expected, 1e-15, "gslmm::expm1(-1e-10)");

  y = gslmm::expm1(0.1);
  y_expected = 0.1051709180756476248117078264902;
  ts.relative(y, y_expected, 1e-15, "gslmm::expm1(0.1)");

  y = gslmm::expm1(-0.1);
  y_expected = -0.09516258196404042683575094055356;
  ts.relative(y, y_expected, 1e-15, "gslmm::expm1(-0.1)");

  y = gslmm::expm1(10.0);
  y_expected = 22025.465794806716516957900645284;
  ts.relative(y, y_expected, 1e-15, "gslmm::expm1(10.0)");

  y = gslmm::expm1(-10.0);
  y_expected = -0.99995460007023751514846440848444;
  ts.relative(y, y_expected, 1e-15, "gslmm::expm1(-10.0)");

  //___________________________________________________________________
  // Test for log1p 
  y = gslmm::log1p(0.0);
  y_expected = 0.0;
  ts.relative(y, y_expected, 1e-15, "gslmm::log1p(0.0)");

  y = gslmm::log1p(1e-10);
  y_expected = 9.9999999995000000000333333333308e-11;
  ts.relative(y, y_expected, 1e-15, "gslmm::log1p(1e-10)");

  y = gslmm::log1p(0.1);
  y_expected = 0.095310179804324860043952123280765;
  ts.relative(y, y_expected, 1e-15, "gslmm::log1p(0.1)");

  y = gslmm::log1p(10.0);
  y_expected = 2.3978952727983705440619435779651;
  ts.relative(y, y_expected, 1e-15, "gslmm::log1p(10.0)");

  //___________________________________________________________________
  // Test for gslmm::hypot
  y = gslmm::hypot(0.0, 0.0);
  y_expected = 0.0;
  ts.relative(y, y_expected, 1e-15, "gslmm::hypot(0.0, 0.0)");

  y = gslmm::hypot(1e-10, 1e-10);
  y_expected = 1.414213562373095048801688e-10;
  ts.relative(y, y_expected, 1e-15, "gslmm::hypot(1e-10, 1e-10)");

  y = gslmm::hypot(1e-38, 1e-38);
  y_expected = 1.414213562373095048801688e-38;
  ts.relative(y, y_expected, 1e-15, "gslmm::hypot(1e-38, 1e-38)");

  y = gslmm::hypot(1e-10, -1.0);
  y_expected = 1.000000000000000000005;
  ts.relative(y, y_expected, 1e-15, "gslmm::hypot(1e-10, -1)");

  y = gslmm::hypot(-1.0, 1e-10);
  y_expected = 1.000000000000000000005;
  ts.relative(y, y_expected, 1e-15, "gslmm::hypot(-1, 1e-10)");

  y = gslmm::hypot(1e307, 1e301);
  y_expected = 1.000000000000499999999999e307;
  ts.relative(y, y_expected, 1e-15, "gslmm::hypot(1e307, 1e301)");

  y = gslmm::hypot(1e301, 1e307);
  y_expected = 1.000000000000499999999999e307;
  ts.relative(y, y_expected, 1e-15, "gslmm::hypot(1e301, 1e307)");

  y = gslmm::hypot(1e307, 1e307);
  y_expected = 1.414213562373095048801688e307;
  ts.relative(y, y_expected, 1e-15, "gslmm::hypot(1e307, 1e307)");

  //___________________________________________________________________
  // Test for acosh 
  y = gslmm::acosh(1.0);
  y_expected = 0.0;
  ts.relative(y, y_expected, 1e-15, "gslmm::acosh(1.0)");

  y = gslmm::acosh(1.1);
  y_expected = 4.435682543851151891329110663525e-1;
  ts.relative(y, y_expected, 1e-15, "gslmm::acosh(1.1)");

  y = gslmm::acosh(10.0);
  y_expected = 2.9932228461263808979126677137742e0;
  ts.relative(y, y_expected, 1e-15, "gslmm::acosh(10.0)");

  y = gslmm::acosh(1e10);
  y_expected = 2.3718998110500402149594646668302e1;
  ts.relative(y, y_expected, 1e-15, "gslmm::acosh(1e10)");

  //___________________________________________________________________
  // Test for asinh
  y = gslmm::asinh(0.0);
  y_expected = 0.0;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(0.0)");

  y = gslmm::asinh(1e-10);
  y_expected = 9.9999999999999999999833333333346e-11;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(1e-10)");

  y = gslmm::asinh(-1e-10);
  y_expected = -9.9999999999999999999833333333346e-11;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(1e-10)");

  y = gslmm::asinh(0.1);
  y_expected = 9.983407889920756332730312470477e-2;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(0.1)");

  y = gslmm::asinh(-0.1);
  y_expected = -9.983407889920756332730312470477e-2;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(-0.1)");

  y = gslmm::asinh(1.0);
  y_expected = 8.8137358701954302523260932497979e-1;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(1.0)");

  y = gslmm::asinh(-1.0);
  y_expected = -8.8137358701954302523260932497979e-1;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(-1.0)");

  y = gslmm::asinh(10.0);
  y_expected = 2.9982229502979697388465955375965e0;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(10)");

  y = gslmm::asinh(-10.0);
  y_expected = -2.9982229502979697388465955375965e0;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(-10)");

  y = gslmm::asinh(1e10);
  y_expected = 2.3718998110500402149599646668302e1;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(1e10)");

  y = gslmm::asinh(-1e10);
  y_expected = -2.3718998110500402149599646668302e1;
  ts.relative(y, y_expected, 1e-15, "gslmm::asinh(-1e10)");

  //___________________________________________________________________
  // Test for atanh 
  y = gslmm::atanh(0.0);
  y_expected = 0.0;
  ts.relative(y, y_expected, 1e-15, "gslmm::atanh(0.0)");

  y = gslmm::atanh(1e-20);
  y_expected = 1e-20;
  ts.relative(y, y_expected, 1e-15, "gslmm::atanh(1e-20)");

  y = gslmm::atanh(-1e-20);
  y_expected = -1e-20;
  ts.relative(y, y_expected, 1e-15, "gslmm::atanh(-1e-20)");

  y = gslmm::atanh(0.1);
  y_expected = 1.0033534773107558063572655206004e-1;
  ts.relative(y, y_expected, 1e-15, "gslmm::atanh(0.1)");

  y = gslmm::atanh(-0.1);
  y_expected = -1.0033534773107558063572655206004e-1;
  ts.relative(y, y_expected, 1e-15, "gslmm::atanh(-0.1)");

  y = gslmm::atanh(0.9);
  y_expected = 1.4722194895832202300045137159439e0;
  ts.relative(y, y_expected, 1e-15, "gslmm::atanh(0.9)");

  y = gslmm::atanh(-0.9);
  y_expected = -1.4722194895832202300045137159439e0;
  ts.relative(y, y_expected, 1e-15, "gslmm::atanh(0.9)");

  //___________________________________________________________________
  // Test for pow_int 
  y = gslmm::pow_2(-3.14);
  y_expected = pow(-3.14, 2.0);
  ts.relative(y, y_expected, 1e-15, "gslmm::pow_2(-3.14)");

  y = gslmm::pow_3(-3.14);
  y_expected = pow(-3.14, 3.0);
  ts.relative(y, y_expected, 1e-15, "gslmm::pow_3(-3.14)");

  y = gslmm::pow_4(-3.14);
  y_expected = pow(-3.14, 4.0);
  ts.relative(y, y_expected, 1e-15, "gslmm::pow_4(-3.14)");

  y = gslmm::pow_5(-3.14);
  y_expected = pow(-3.14, 5.0);
  ts.relative(y, y_expected, 1e-15, "gslmm::pow_5(-3.14)");

  y = gslmm::pow_6(-3.14);
  y_expected = pow(-3.14, 6.0);
  ts.relative(y, y_expected, 1e-15, "gslmm::pow_6(-3.14)");

  y = gslmm::pow_7(-3.14);
  y_expected = pow(-3.14, 7.0);
  ts.relative(y, y_expected, 1e-15, "gslmm::pow_7(-3.14)");

  y = gslmm::pow_8(-3.14);
  y_expected = pow(-3.14, 8.0);
  ts.relative(y, y_expected, 1e-15, "gslmm::pow_8(-3.14)");

  y = gslmm::pow_9(-3.14);
  y_expected = pow(-3.14, 9.0);
  ts.relative(y, y_expected, 1e-15, "gslmm::pow_9(-3.14)");

  for (int n = -9; n < 10; n++) {
    y = gslmm::pow_int(-3.14, n);
    y_expected = pow(-3.14, n);
    ts.relative(y, y_expected, 1e-15, "gslmm::pow_n(-3.14,%d)", n);
  }

  //___________________________________________________________________
  // Test for ldexp
  y = gslmm::ldexp(M_PI, -2);
  y_expected = M_PI_4;
  ts.relative(y, y_expected, 1e-15, "gslmm::ldexp(pi,-2)");

  y = gslmm::ldexp(1.0, 2);
  y_expected = 4.000000;
  ts.relative(y, y_expected, 1e-15, "gslmm::ldexp(1.0,2)");

  y = gslmm::ldexp(0.0, 2);
  y_expected = 0.0;
  ts.relative(y, y_expected, 1e-15, "gslmm::ldexp(0.0,2)");

  //___________________________________________________________________
  // Test for frexp 
  y = gslmm::frexp(M_PI, e);
  y_expected = M_PI_4;
  e_expected = 2;
  ts.relative(y, y_expected, 1e-15, "gslmm::frexp(pi) fraction");
  ts.test(e, e_expected, "gslmm::frexp(pi) exponent");

  y = gslmm::frexp(2.0, e);
  y_expected = 0.5;
  e_expected = 2;
  ts.relative(y, y_expected, 1e-15, "gslmm::frexp(2.0) fraction");
  ts.test(e, e_expected, "gslmm::frexp(2.0) exponent");

  y = gslmm::frexp(1.0 / 4.0, e);
  y_expected = 0.5;
  e_expected = -1;
  ts.relative(y, y_expected, 1e-15, "gslmm::frexp(0.25) fraction");
  ts.test(e, e_expected, "gslmm::frexp(0.25) exponent");

  y = gslmm::frexp(1.0 / 4.0 - 4.0 * gslmm::type_trait<double>::epsilon(), e);
  y_expected = 0.999999999999996447;
  e_expected = -2;
  ts.relative(y, y_expected, 1e-15, "gslmm::frexp(0.25-eps) fraction");
  ts.test(e, e_expected, "gslmm::frexp(0.25-eps) exponent");


  //___________________________________________________________________
  // Test for approximate floating point comparison */
  double x = M_PI;
  y = 22.0 / 7.0;

  //___________________________________________________________________
  // test the basic function 
  for (int i = 0; i < 10; i++) {
    double tol = pow(10, -i);
    int    res = gslmm::compare(x, y, tol);
    ts.test(res, -(i >= 4 ? 1 : 0), "gslmm::compare(%.5f,%.5f,%g)", x, y, tol);
  }

  for (int i = 0; i < 10; i++) {
    double tol = pow(10, -i);
    int    res = gslmm::compare(y, x, tol);
    ts.test(res, (i >= 4 ? 1 : 0), "gslmm::compare(%.5f,%.5f,%g)", y, x, tol);
  }
    

  // #ifdef HAVE_IEEE_COMPARISONS
  //___________________________________________________________________
  // Test for isinf, isnan, finite

  int    s;  
  bool   b;
  double zero = 0.0;
  double one = 1.0;
  double inf = exp(1.0e10);
  double nan = inf / inf;

  s = gslmm::is_infinite(zero);
  ts.test(s, 0, "gslmm::is_infinite(0)");

  s = gslmm::is_infinite(one);
  ts.test(s, 0, "gslmm::is_infinite(1)");

  s = gslmm::is_infinite(inf);
  ts.test(s, 1, "gslmm::is_infinite(inf)");

  s = gslmm::is_infinite(-inf);
  ts.test(s, -1, "gslmm::is_infinite(-inf)");

  s = gslmm::is_infinite(nan);
  ts.test(s, 0, "gslmm::is_infinite(nan)");


  b = gslmm::is_not_a_number(zero);
  ts.test(b, false, "gslmm::is_not_a_number(0)");

  b = gslmm::is_not_a_number(one);
  ts.test(b, false, "gslmm::is_not_a_number(1)");

  b = gslmm::is_not_a_number(inf);
  ts.test(b, false, "gslmm::is_not_a_number(inf)");

  b = gslmm::is_not_a_number(nan);
  ts.test(b,  true, "gslmm::is_not_a_number(nan)");


  b = gslmm::is_finite(zero);
  ts.test(b,  true, "gslmm::is_finite(0)");

  b = gslmm::is_finite(one);
  ts.test(b,  true, "gslmm::is_finite(1)");

  b = gslmm::is_finite(inf);
  ts.test(b, false, "gslmm::is_finite(inf)");

  b = gslmm::is_finite(nan);
  ts.test(b, false, "gslmm::is_finite(nan)");
  // #endif

  //___________________________________________________________________
  x = gslmm::divide(2.0, 3.0);
  ts.relative(x, 2.0 / 3.0, 4 * gslmm::type_trait<double>::epsilon(), 
	      "gslmm::divide(2,3)");

  return ts.summary() ? 0 : 1;
}

//______________________________________________________________________
//
// EOF
//  
