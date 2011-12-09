// -*- mode: c++ -*-
//
// $Id: gslmm-util.hh.in,v 1.3 2003-03-22 04:35:39 cholm Exp $ 
//  
//  gslmm::complex
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
#ifndef GSLMM_util
#define GSLMM_util
#define GSLMM_VERSION_STRING "0.12"
#define GSLMM_VERSION        0.12

/** @file   gslmm-util.hh
    @author Christian Holm
    @date   Wed Sep 18 02:11:03 2002
    @brief  Various utility macros, etc.*/

/** @defgroup misc Miscellaneous */
/** Namespace of all the GSL-- classes. */
namespace gslmm
{  
  //_____________________________________________________________________
  /** A locking policy useful for singlethreaded applications. 
      This class is only useful for single threaded applications.  If
      you need threads, then you should define an appropiate class,
      and pass that as template parameter to the relevant classes. 
      @ingroup misc 
  */
  class single_thread_lock
  {
  public:
    /// Acquire this lock. 
    void lock() {}
    /// Release this lock. 
    void unlock() {}
  };

  //_____________________________________________________________________
  /** Class to ease the locking. 
      The argument to the constructor must be some kind of lock.  When
      the object is created (constructed) the lock is aquired, and
      when the object goes out of scope (destructed), the lock is
      released.  
      @ingroup misc 
      @param Lock The type of lock to lock. 
  */
  template <typename Lock>
  struct guard 
  {
    /// The lock.
    Lock& _lock;
    /** CTOR.  Acquires the lock 
	@param lock The lock to acquire. */
    guard(Lock& lock) : _lock(lock) { _lock.lock(); }
    /** DTOR. Releases the lock. */
    ~guard() { _lock.unlock(); }
  };
}

#ifndef GSLMM_LOCK
# define GSLMM_LOCK gslmm::single_thread_lock
#endif

#endif
//____________________________________________________________________
//
// EOF
//

