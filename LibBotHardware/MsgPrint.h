#ifndef ERRORMSG_H
#define ERRORMSG_H

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

/*
 * Error/warning/info print macros.
 *
 * Redefined to avoid player dependencies. Makes it easier to switch to another
 * log system (e.g. one thats not that ugly).
 */

// Print/log an error
#define ERRORPRINT(_MSG_) fprintf( stderr, "%s\n", _MSG_)
#define ERRORPRINT1(_MSG_,_A_) {fprintf( stderr, _MSG_, _A_);std::cerr << std::endl;}
#define ERRORPRINT2(_MSG_,_A_,_B_) {fprintf( stderr, _MSG_, _A_, _B_);std::cerr << std::endl;}
#define ERRORPRINT3(_MSG_,_A_,_B_,_C_) {fprintf( stderr, _MSG_, _A_, _B_, _C_);std::cerr << std::endl;}
#define ERRORPRINT4(_MSG_,_A_,_B_,_C_,_D_) {fprintf( stderr, _MSG_, _A_, _B_, _C_, _D_);std::cerr << std::endl;}

// Print/log a warning
#define WARNPRINT(_MSG_) printf( "%s\n", _MSG_ )
#define WARNPRINT1(_MSG_,_A_) {printf( _MSG_, _A_ );std::cout << std::endl;}
#define WARNPRINT2(_MSG_,_A_,_B_) {printf( _MSG_, _A_, _B_ );std::cout << std::endl;}
#define WARNPRINT3(_MSG_,_A_,_B_,_C_) {printf( _MSG_, _A_, _B_, _C_ );std::cout << std::endl;}
#define WARNPRINT4(_MSG_,_A_,_B_,_C_,_D_) {printf( _MSG_, _A_, _B_, _C_, _D_ );std::cout << std::endl;}

// Print/log an info
#define INFOPRINT(_MSG_) printf( "%s\n", _MSG_ )
#define INFOPRINT1(_MSG_,_A_) {printf( _MSG_, _A_ );std::cout << std::endl;}
#define INFOPRINT2(_MSG_,_A_,_B_) {printf( _MSG_, _A_, _B_ );std::cout << std::endl;}
#define INFOPRINT3(_MSG_,_A_,_B_,_C_) {printf( _MSG_, _A_, _B_, _C_ );std::cout << std::endl;}
#define INFOPRINT4(_MSG_,_A_,_B_,_C_,_D_) {printf( _MSG_, _A_, _B_, _C_, _D_ );std::cout << std::endl;}

// Debug output (add file and line number)
#define DEBUGOUT (std::cout << __FILE__ << ":" << __LINE__ << ": " )

#endif // ERRORMSG_H
