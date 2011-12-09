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

#ifndef KUTILITY_CONVOLUTION_OPENCV_TCC
#define KUTILITY_CONVOLUTION_OPENCV_TCC

#if defined(WITH_OPENCV) && defined(WITH_OPENCV_EXTRAS)

#include "cv.h"
#include "highgui.h"

namespace kutility
{
   inline void conv_horizontal( float* image, int h, int w, float* kernel, int ksize )
   {
      CvMat cvI; cvInitMatHeader(&cvI, h, w, CV_32FC1, (float*)image);
      CvMat cvK; cvInitMatHeader(&cvK, 1, ksize, CV_32FC1, (float*)kernel );
      cvFilter2D( &cvI, &cvI, &cvK );
   }
   inline void conv_horizontal( double* image, int h, int w, double* kernel, int ksize )
   {
      CvMat cvI; cvInitMatHeader(&cvI, h, w, CV_64FC1, (double*)image);
      CvMat cvK; cvInitMatHeader(&cvK, 1, ksize, CV_64FC1, (double*)kernel );
      cvFilter2D( &cvI, &cvI, &cvK );
   }

   inline void conv_vertical( float* image, int h, int w, float* kernel, int ksize )
   {
      CvMat cvI; cvInitMatHeader(&cvI, h, w, CV_32FC1, (float*)image);
      CvMat cvK; cvInitMatHeader(&cvK, ksize, 1, CV_32FC1, (float*)kernel );
      cvFilter2D( &cvI, &cvI, &cvK );
   }

   inline void conv_vertical( double* image, int h, int w, double* kernel, int ksize )
   {
      CvMat cvI; cvInitMatHeader(&cvI, h, w, CV_64FC1, (double*)image);
      CvMat cvK; cvInitMatHeader(&cvK, ksize, 1, CV_64FC1, (double*)kernel );
      cvFilter2D( &cvI, &cvI, &cvK );
   }

   template<typename T> inline
   void convolve_sym_( T* image, int h, int w, T* kernel, int ksize )
   {
      conv_horizontal( image, h, w, kernel, ksize );
      conv_vertical  ( image, h, w, kernel, ksize );
   }
}

#endif

#endif
