//
// $Id: test_suite.hh,v 1.19 2006-05-01 14:25:48 cholm Exp $ 
//  
//  gslmm::test_suite
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
#ifndef GSL_test_suite
#define GSL_test_suite
#ifndef __STRING__
#include <string>
#endif
#ifndef __IOSTREAM__
#include <iostream>
#endif
#ifndef __IOMANIP__
#include <iomanip>
#endif
#ifndef __CSTDARG__
#include <cstdarg>
#endif
#ifndef __CSTDIO__
#include <cstdio>
#endif
#ifndef __CMATH__
#include <cmath>
#endif
#include <stdlib.h>
#ifndef __GSL_VERSION_H__
#include <gsl/gsl_version.h>
#endif
#ifndef GSLMM_util
#include <gslmm/gslmm-util.hh>
#endif
#ifndef OPTIONMM_command_line
# include <optionmm/command_line.hh>
#endif
#ifdef HAVE_CONFIG_H
# include "config.hh"
#else
# define VERSION "?.?"
#endif

/** @file   test_suite.hh
    @author Christian Holm
    @date   Fri Sep 20 13:40:23 2002 
    @brief  Declaration of test suite class. */
/** @defgroup test Test suite handling */
namespace gslmm
{
  /** @class test_suite test_suite.hh <gslmm/test/test_suite.hh>
      @brief Utility class to use in test suits.

      The idea is that one creates an object of this class, and then
      do operations in the program.  Then, one passes the result of
      the operation, as well as the expected result of the operation,
      to the appropiate member function of this class. The object then
      keeps a score of how many operations gave the expected result.

      Turning on the verbosity will print out descriptions of the
      tests evaluated, along with information on whether the test
      passed or failed. 

      Various tests are defined.  They are explained in the member
      function documentation. 

      A simple example could be
      @code 
      #include <gslmm/test/test_suite.hh>
      #include <gslmm/math/type_trait.hh>

      int main(int argc, char** argv) 
      { 
      gslmm::test_suite ts;

      for (int i = 1; i < argc; i++) {
        if (!argv[i][0] || argv[i][0] != '-' || !argv[i][1]) continue;
          switch (argv[i][1]) {
          case 'h':  
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            return 0;
          case 'v': ts.verbose(); break;
          }
        }

	double x = M_PI;
	y = 22.0 / 7.0;

	//___________________________________________________________________
	// test the basic function 
	for (int i = 0; i < 10; i++) {
	  double tol = pow(10, -i);
	  int    res = gslmm::compare(x, y, tol);
	  ts.test(res, -(i >= 4 ? 1 : 0), 
	          "gslmm::compare(%.5f,%.5f,%g)", x, y, tol);
	}
	
        return ts.summary() ? 0 : 1;
      }
      @endcode 
      The above will print 
      @verbatim
      PASS: gslmm::compare(3.14159,3.14286,1)
      PASS: gslmm::compare(3.14159,3.14286,0.1)
      PASS: gslmm::compare(3.14159,3.14286,0.01)
      PASS: gslmm::compare(3.14159,3.14286,0.001)
      PASS: gslmm::compare(3.14159,3.14286,0.0001)
      PASS: gslmm::compare(3.14159,3.14286,1e-05)
      PASS: gslmm::compare(3.14159,3.14286,1e-06)
      PASS: gslmm::compare(3.14159,3.14286,1e-07)
      PASS: gslmm::compare(3.14159,3.14286,1e-08)
      PASS: gslmm::compare(3.14159,3.14286,1e-09)
      10 tests, 10 passed, 0 failed.
      All tests passed successfully
      @endverbatim
      if the verbose output was turned on.
      @ingroup test
   */
  class test_suite
  {
  protected:
    static test_suite* _instance;
    optionmm::command_line _cl;
    /// The verbosity level.
    optionmm::bool_option  _verbose;
    /// The total number of tests so far.
    int _tests;
    /// The number of passed tests so far.
    int _passed;
    /// The number of failed tests so far. 
    int _failed;
    /// Buffer for output messages
    char _buf[1024];

    /** Utility function to increment counters etc. 
	@param status The status of a test. */ 
    bool store(bool status);
    /** Utility function to print result for double tests. 
	@param status Status of test. 
	@param result The result. 
	@param expected The exprected result. */
    template <typename Type, typename Type1>
    void print(bool status, const Type& result, const Type1& expected);
  public:
    /** Constructor.
     */
    test_suite(const std::string& n, int& argc, char** argv);
    /** Destructor. 
     */
    virtual ~test_suite() {}

    static test_suite& instance() 
    {
      return *_instance;
    }
    /** Whether we're verbose */
    bool is_verbose() const { return _verbose.value(); }
    
    /** Show a banner */ 
    void banner() const;

    //@{
    /// @name Tests
    /** Test a truth value.
	The test is passed if @a status evaluates to true. 
	@param status      The truth value.
	@param description What you expect. 
        @return true if test succeded, false otherwise. */
    bool check(bool status, const char* description, ...);
    /** Test a return status. 
	The test is passed if @a status is 0. 
	@param status      The result of the computations.
	@param description What you expect. 
        @return true if test succeded, false otherwise. */
    bool status(int status, const char* description, ...);
    /** Relative error test. 
	The test is passed if 
	@f$ |result - expected|/|expected| \leq error @f$
	@param result      The result of the computations.
	@param expected    The expected result. 
	@param error       The relatvie tolerance. 
	@param description What you expect. 
        @return true if test succeded, false otherwise. */
    bool relative(double result, double expected, double error, 
		  const char* description, ...);
    /** Absolute error test. 
	The test is passed if 
	@f$ |result - expected| \leq error @f$
	@param result      The result of the computations.
	@param expected    The expected result. 
	@param error       The absolute tolerance. 
	@param description What you expect. 
        @return true if test succeded, false otherwise. */
    template <typename Type>
    bool absolute(const Type& result, const Type& expected, 
		  const Type& error, const char* description, ...);
    /** Factor error test. 
	The test is passed if 
	@f$ f \leq error \wedge f \leq 1 / error@f$, where 
	@f$ f = result / expected @f$
	@param result      The result of the computations.
	@param expected    The expected result. 
	@param error       The factor tolerance. 
	@param description What you expect. 
        @return true if test succeded, false otherwise. */
    bool factor(double result, double expected, double error, 
		const char* description, ...);

    /** Range error test. 
	Test is passed if @f$ min < result < max@f$
	@param result      The result of the computations
	@param min         The lower limit
	@param max         The upper limit.
	@param description What you expect. 
        @return true if test succeded, false otherwise. */
    template <typename Type> 
    bool range(const Type& result, const Type& min, const Type& max,
	       const char* description, ...);
    
    /** Compare arbitriary types. 
	Type must define the @c == operator, and the put-to operator
	for streams (@c operator<<(ostream&,const @c Type&)).
	@param result      The result of the computations.
	@param expected    The expected result. 
	@param description What you expect. 
        @return true if test succeded, false otherwise. */
    template <typename Type, typename Type1>
    bool test(const Type& result, const Type1& expected, 
	      const char* description, ...);
    //@}
    
    //@{
    /// @name Status of test suite 
    /** Write a summary of the tests. 
	@return true if all tests succeded, false otherwise. */
    bool summary();
    /** Get the number of tests. 
	@return the number of tests done so far. */
    int tests() const { return _tests; }
    /** Get the number of passed tests. 
	@return the number of passed tests done so far. */
    int passed() const { return _passed; }
    /** Get the number of failed tests. 
	@return the number of failed tests done so far. */
    int failed() const { return _failed; }
    /** Print a message */ 
    void message(const char* format, ...);
    //@}    
  };

#ifndef NO_TESTSUITE_INSTANCE
  test_suite* test_suite::_instance = 0;
#endif

  //__________________________________________________________________
  inline 
  test_suite::test_suite(const std::string& n, int& argc, char** argv)
    : _cl(std::string("GSL-- test of module \"")+n+std::string("\""), 
	  VERSION, "Copyright (c) 2004 Christian Holm Christensen", "", 
	  argc, argv), 
      _verbose('v', "verbose", "To be verbose or not", false),
      _tests(0), _passed(0), _failed(0)
  {
	std::cout<<"test_suite in" << std::endl;
    _instance = this;
    _cl.add(_verbose);
    if (!_cl.process()) {
      std::cerr << "Bad commandline" << std::endl;
      exit(1);
    }
    if (_cl.help()) {
      banner();
      exit(0);
    }
    if (_cl.version()) exit(0);
  }
  //__________________________________________________________________
  inline void 
  test_suite::banner() const
  {
    if (_verbose.value()) return;
    
    std::cout << "Test suite for GSL-- version " 
	      << GSLMM_VERSION_STRING << std::endl
	      << "Compiled against GSL version " 
	      << GSL_VERSION << std::endl << std::endl
	      << "Copyright (c) 2003 Christian Holm <cholm@nbi.dk>\n"
	      << "This is free software; see the source for copying  "
	      << "conditions.  There is NO\nwarranty; not even for "
	      << "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."
	      << std::endl;
  }
  
  //__________________________________________________________________
  inline bool 
  test_suite::store(bool status)
  {
    _tests++; 
    if (status) _passed++; 
    else        _failed++; 
    return status;
  }
  //____________________________________________________________________
  template <typename Type, typename Type1>
  inline void test_suite::print(bool status, 
				const Type& result, 
				const Type1& expected)
  {
    if (status) std::cout << "PASS: ";
    else        std::cout << "FAIL: ";
    std::cout << _buf;
    size_t prec = std::cout.precision();
    if (!status) 
      std::cout << "\n      (" << std::scientific << std::setw(20)
		<< std::setprecision(14) << result << " vs " 
		<< std::setw(20) << expected << " expected)";
    std::cout << std::setprecision(prec) << std::endl;
    std::cout.setf(std::ios_base::fmtflags(0),std::ios_base::floatfield);
  }
#define _GSLMM_TEST(s,r,e,f) \
  do { if (_verbose.value()) { va_list ap; va_start(ap, f); \
      vsprintf(_buf, f, ap); va_end(ap); print(s,r,e); } } while(false)
  //__________________________________________________________________
  inline bool 
  test_suite::check(bool val, const char* description, ...) 
  {
    store(val);
    _GSLMM_TEST(val,val,true,description);
    return val;
  }
  //____________________________________________________________________
  inline bool 
  test_suite::status(int val, const char* description, ...) 
  {
    bool ret = store(val == 0);
    _GSLMM_TEST(ret,val,0,description);
    return ret;
  }
  //__________________________________________________________________
  inline bool 
  test_suite::relative(double result, double expected, 
		       double error, const char* description, ...) 
  {
    bool ret = false;
    if (expected != 0) 
      ret = fabs(result - expected) / fabs(expected) <= error;
    else 
      ret = fabs(result) <= error; 
    store(ret);
    _GSLMM_TEST(ret,result,expected,description);
    return ret;
  }
  //__________________________________________________________________
  template <typename T>
  inline bool 
  test_suite::absolute(const T& result, const T& expected, 
		       const T& error, const char* description, ...) 
  {
    bool ret = store(fabs(result - expected) <= error);
    _GSLMM_TEST(ret,result,expected,description);
    return ret;
  }
  //____________________________________________________________________
  inline bool 
  test_suite::factor(double result, double expected, 
		     double error, const char* description, ...) 
  {
    bool ret;
    if (result == expected) ret = true; 
    else if (expected == 0) ret = (result <= expected && result >= expected);
    else ret = ((result / expected <= error) &&
		(result / expected >= 1. / error));
    store(ret);
    _GSLMM_TEST(ret,result,expected,description);
    return ret;
  }
  //__________________________________________________________________
  template <typename Type>
  inline bool 
  test_suite::range(const Type& result, 
		    const Type& min,
		    const Type& max,
		    const char* description, ...) 
  {
    bool ret = store(result < max && result > min); 
    Type expected = (max - min) / 2 + min;
    _GSLMM_TEST(ret,result,expected,description);
    return ret;
  }
  //__________________________________________________________________
  template <typename Type, typename Type1>
  inline bool 
  test_suite::test(const Type& result, 
		   const Type1& expected, 
		   const char* description, ...) 
  {
    bool ret = store(result == expected); 
    _GSLMM_TEST(ret,result,expected,description);
    return ret;
  }
  //____________________________________________________________________
  inline void 
  test_suite::message(const char* format, ...)
  {
    if (!_verbose.value()) return;
    va_list ap; 
    va_start(ap, format);
    vsprintf(_buf, format, ap); 
    va_end(ap); 
    std::cout << "NOTE: " << _buf << std::endl;
  }
    
  //____________________________________________________________________
  inline bool 
  test_suite::summary() 
  {
    if (_verbose.value()) 
      std::cout << _tests << " tests, " << _passed << " passed, "
		<< _failed << " failed." << std::endl;
  
    if (_failed != 0) {
      if (_verbose.value()) 
	std::cout << _failed << " TEST" << (_failed == 1 ? "" : "S") 
		  << " FAILED." << std::endl;
      return false;
    }
    if (_tests != _passed + _failed) {
      if (_verbose.value()) 
	std::cout << "TEST RESULTS DO NOT ADD UP " << _tests 
		  << " != " << _passed << " + " << _failed << std::endl;
      return false;
    }
    if (_passed == _tests) 
      if (_verbose.value()) 
	std::cout << "All tests passed successfully" << std::endl;
    return true;
  }
#undef _GSLMM_TEST
}

#endif
//____________________________________________________________________
//
// EOF
//
