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

#ifndef KUTILITY_INTERACTION_H
#define KUTILITY_INTERACTION_H

#include "kutility/kutility.def"

namespace kutility
{
   /// prints an error message and exits with exit code "code".
   /// if no code is given, it exits with 1.
   void error( string str1, int code=1 );

   /// prints an error message concataneting strings and exits with
   /// exit code "code". if no code is given, it exits with 1.
   void error( string str1, string str2, int code=1 );

   /// prints an error message concataneting strings and exits with
   /// exit code "code". if no code is given, it exits with 1.
   void error( string str1, string str2, string str3, int code=1 );

   /// prints the warning message "str1"
   void warning( string str1, string str2="", string str3="" );

   /// prints a message with separators(dashes by deafult) above and below.
   void major_message( string str1, string str2="", string str3="", string sep="-" );

   /// prints a message "msg"
   void message( string str1, string str2="", string str="" );

   /// prints a number and its name
   template<class T>
   void message( string str, T num )
   {
      std::cout<<str<<" : "<<num<<std::endl;
   }

   /// prints a progress message giving the percent of completion.
   /// estimates the remaining time if the time-elasped is given
   template<class T1, class T2> inline
   void progress(T1 state, T2 range, int freq, time_t elapsed=-1)
   {
      if( ((int)(state)) % freq == 0 )
      {
         std::cout.width(5);
         std::cout.precision(4);
         double percent = ((double)(state))/((double)(range));
         std::cout<<"completed: "<<100*percent<<"%";

         double eta;
         if( elapsed != -1 )
         {
            eta = ((double)elapsed)/percent;
            std::cout<<"\tremaining: "<<(double)(eta-elapsed)<<"s\t total: "<<eta<<"s";
         }
         std::cout<<"\n";
      }
   }

   /// displays an array in matrix form of r x c (c=1 by default). it
   /// has various options to affect the display format. "no_zero", if
   /// set true, prints white spaces instead of zeros, "legend"=true
   /// enables displaying the index legend, "precision" sets the
   /// precision of the displayed data thru cout.precision, "width"
   /// sets the horizontal spacing of the number and "sep" is the
   /// seperation character of the numbers.
   template<class T> inline
   void display( T* data, int r, int c=1, bool no_zero=false, bool legend=false, int precision=3, int width=4, string sep="\t")
   {
      cout.width(width);
      cout.fill(' ');
      cout.precision(precision);

      int i,j;
      if( legend )
      {
         cout<<"\t"<<"  ";
         cout.setf( ios_base::right);
         for(j=0; j<c; j++)
         {
            cout.width(width);
            cout.precision(precision);
            cout<<j<<sep;
         }
         cout<<endl;
         for(j=0; j<140; j++)
         {
            cout<<'.';
         }
      }
      cout<<endl;
      for(i=0; i<r; i++)
      {
         if( legend )
         {
            cout.setf( ios_base::right );
            cout.width(width);
            cout.precision(precision);
            cout<<i<<"\t"<<": ";
         }
         cout.setf( ios_base::right );
         for(j=0; j<c; j++)
         {
            cout.width(width);
            cout.setf( ios_base::right );
            cout.precision(precision);

            if( no_zero && data[i*c+j] == 0 )
               cout<<" "<<sep;
            else
               cout<<data[i*c+j]<<sep;
         }
         cout<<endl;
      }
      cout<<endl;
   }

   /// displays a matrix form of r x c (c=1 by default). it has
   /// various options to affect the display format. "no_zero", if set
   /// true, prints white spaces instead of zeros, "legend"=true
   /// enables displaying the index legend, "precision" sets the
   /// precision of the displayed data thru cout.precision, "width"
   /// sets the horizontal spacing of the number and "sep" is the
   /// seperation character of the numbers.
   template<class T> inline
   void display( T** data, int r, int c=1,  bool no_zero=false, bool legend=false, int precision=3, int width=4, char* sep="\t")
   {
      cout.width(width);
      cout.fill(' ');
      cout.precision(precision);

      int i,j;
      if( legend )
      {
         cout<<"\t"<<"  ";
         cout.setf( ios_base::right);
         for(j=0; j<c; j++)
         {
            cout.width(width);
            cout.precision(precision);
            cout<<j<<sep;
         }
         cout<<endl;
         for(j=0; j<140; j++)
         {
            cout<<'.';
         }
      }
      cout<<endl;
      for(i=0; i<r; i++)
      {
         if( legend )
         {
            cout.setf( ios_base::right );
            cout.width(width);
            cout.precision(precision);
            cout<<i<<"\t"<<": ";
         }
         cout.setf( ios_base::right );
         for(j=0; j<c; j++)
         {
            cout.width(width);
            cout.setf( ios_base::right );
            cout.precision(precision);

            if( no_zero && data[i][j] == 0 )
               cout<<" "<<sep;
            else
               cout<<data[i][j]<<sep;

         }
         cout<<endl;
      }
      cout<<endl;
   }
}
#endif
