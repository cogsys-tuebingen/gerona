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

#ifndef KUTILITY_FILEIO_H
#define KUTILITY_FILEIO_H

#include "kutility/general.h"

namespace kutility
{
   enum data_types {TYPE_CHAR, TYPE_FLOAT, TYPE_DOUBLE, TYPE_INT};

   // ascii
   template<typename T> inline void save_ascii( ofstream& fout, T* data, int h, int w, int nb, int type );
   template<typename T> inline void save_ascii( string filename, T* data, int h, int w, int nb, int type );
   template<typename T> inline void load_ascii( ifstream& fin, T* &data, int &h, int &w, int &nb );
   template<typename T> inline void load_ascii( string filename, T* &data, int &h, int &w, int &nb );

   // binary
   template<class T> inline void save_binary(ofstream& fout, T* data, int h, int w, int nb, int type );
   template<class T> inline int  save_binary(string filename, T* data, int h, int w, int nb, int type );
   inline int load_binary(ifstream &fin, float*  &data, int &h, int &w, int &nb );
   inline int load_binary(ifstream &fin, int*    &data, int &h, int &w, int &nb );
   inline int load_binary(ifstream &fin, double* &data, int &h, int &w, int &nb );
   inline int load_binary(ifstream &fin, char*   &data, int &h, int &w, int &nb );
   template<typename T> inline int load_binary(string filename, T* &data, int &h, int &w, int &nb );

   template<class T> inline void save_plain(ofstream& fout, T* data, int sz );
   template<class T> inline void save_plain(ofstream& fout, T* data, int rs, int cs );
   template<class T> inline void save(string filename, T* data, int sz );
   template<class T> inline void save(string filename, T* data, int rs, int cs, bool append=true);

   template<class T> inline int load( ifstream& fin, T* &out, int size=-1 );
   template<class T> inline int load( string filename, T* &out, int size=-1 );
   inline void* load_array( string filename, int size, int type=1 );

   #include "fileio.tcc"
}

#endif

