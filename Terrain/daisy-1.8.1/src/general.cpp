//////////////////////////////////////////////////////////////////////////
// Software License Agreement (BSD License)                             //
//                                                                      //
// Copyright (c) 2009                                                   //
// Engin Tola                                                           //
// web   : http://cvlab.epfl.ch/~tola                                   //
// email : engin.tola@epfl.ch                                           //
//                                                                      //
// All rights reserved.                                                 //
//                                                                      //
// Redistribution and use in source and binary forms, with or without   //
// modification, are permitted provided that the following conditions   //
// are met:                                                             //
//                                                                      //
//  * Redistributions of source code must retain the above copyright    //
//    notice, this list of conditions and the following disclaimer.     //
//  * Redistributions in binary form must reproduce the above           //
//    copyright notice, this list of conditions and the following       //
//    disclaimer in the documentation and/or other materials provided   //
//    with the distribution.                                            //
//  * Neither the name of the EPFL nor the names of its                 //
//    contributors may be used to endorse or promote products derived   //
//    from this software without specific prior written permission.     //
//                                                                      //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS  //
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT    //
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS    //
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE       //
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  //
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, //
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER     //
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT   //
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN    //
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE      //
// POSSIBILITY OF SUCH DAMAGE.                                          //
//                                                                      //
// See licence.txt file for more details                                //
//////////////////////////////////////////////////////////////////////////

#include "kutility/general.h"

namespace kutility
{
   char* itoa(int value, char*  str, int radix)
   {
      int  rem = 0;
      int  pos = 0;
      char ch  = '!' ;
      do
      {
         rem    = value % radix ;
         value /= radix;
         if ( 16 == radix )
         {
            if( rem >= 10 && rem <= 15 )
            {
               switch( rem )
               {
               case 10:
                  ch = 'a' ;
                  break;
               case 11:
                  ch ='b' ;
                  break;
               case 12:
                  ch = 'c' ;
                  break;
               case 13:
                  ch ='d' ;
                  break;
               case 14:
                  ch = 'e' ;
                  break;
               case 15:
                  ch ='f' ;
                  break;
               }
            }
         }
         if( '!' == ch )
         {
            str[pos++] = (char) ( rem + 0x30 );
         }
         else
         {
            str[pos++] = ch ;
         }
      }while( value != 0 );
      str[pos] = '\0' ;
      return strrev(str);
   }

   //strrev the standard way
   // the following directives to make the code portable
   // between windows and Linux.
   char* strrev(char* szT)
   {
      if ( !szT )                 // handle null passed strings.
         return NULL;
      int i = strlen(szT);
      int t = !(i%2)? 1 : 0;      // check the length of the string .
      for(int j = i-1 , k = 0 ; j > (i/2 -t) ; j-- )
      {
         char ch  = szT[j];
         szT[j]   = szT[k];
         szT[k++] = ch;
      }
      return szT;
   }

}

