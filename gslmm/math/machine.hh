//
// $Id: machine.hh,v 1.7 2006-04-26 10:06:04 cholm Exp $ 
//  
//  gslmm::machine
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
#ifndef GSL_machine
#define GSL_machine
#ifndef __GSL_MACHINE_H__
#include  <gsl/gsl_machine.h>
#endif

/** @file   math/machine.hh
    @author Christian Holm
    @date   Wed Mar 12 14:55:57 2003
    @brief  Machine specific constants */


/** @defgroup precision Precision of numbers */
namespace gslmm
{
  /** @class machine machine.hh <gslmm/math/machine.hh>
      @brief Machine precision constants. 
      @ingroup precision 
  */
  struct machine 
  {
    /// Smallest difference representable by a double variable,
    /// @f$\epsilon_{\mbox{double}}@f$. 
    static const double epsilon()          { return GSL_MACH_EPS; }
    /// @f$\sqrt{\epsilon_{\mbox{double}}}@f$.
    static const double sqrt_of_epsilon()  { return GSL_SQRT_MACH_EPS; }
    /// @f$\sqrt[3]{\epsilon_{\mbox{double}}}@f$.
    static const double root3_of_epsilon() { return GSL_ROOT3_MACH_EPS; }
    /// @f$\sqrt[4]{\epsilon_{\mbox{double}}}@f$.
    static const double root4_of_epsilon() { return GSL_ROOT4_MACH_EPS; }
    /// @f$\sqrt[5]{\epsilon_{\mbox{double}}}@f$.
    static const double root5_of_epsilon() { return GSL_ROOT5_MACH_EPS; }
    /// @f$\sqrt[6]{\epsilon_{\mbox{double}}}@f$.
    static const double root6_of_epsilon() { return GSL_ROOT6_MACH_EPS; }
    /// @f$\log{\epsilon_{\mbox{double}}}@f$.
    static const double log_of_epsilon()   { return GSL_LOG_MACH_EPS; }
  };
}

#endif
//____________________________________________________________________
//
// EOF
//
