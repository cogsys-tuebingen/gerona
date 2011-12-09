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

#ifndef KUTILITY_CONVOLUTION_H
#define KUTILITY_CONVOLUTION_H

#if defined(WITH_OPENCV) && defined(WITH_OPENCV_EXTRAS)
   #include "kutility/convolution_opencv.h"
#else
   #include "kutility/convolution_default.h"
#endif

namespace kutility
{
   inline void convolve_sym( float* image, int h, int w, float* kernel, int ksize, float* out=NULL )
   {
      if( out == NULL ) out = image;
      else memcpy( out, image, sizeof(float)*h*w );

      if( h == 240 && w ==  320 ) { convolve_sym_(out, 240, 320, kernel, ksize); return; }
      if( h == 480 && w ==  640 ) { convolve_sym_(out, 480, 640, kernel, ksize); return; }
      if( h == 512 && w ==  512 ) { convolve_sym_(out, 512, 512, kernel, ksize); return; }
      if( h == 512 && w ==  768 ) { convolve_sym_(out, 512, 768, kernel, ksize); return; }
      if( h == 600 && w ==  800 ) { convolve_sym_(out, 600, 800, kernel, ksize); return; }
      if( h == 768 && w == 1024 ) { convolve_sym_(out, 768, 1024, kernel, ksize); return; }
      if( h == 1024 && w == 768 ) { convolve_sym_(out, 1024, 768, kernel, ksize); return; }
      if( h == 256 && w ==  256 ) { convolve_sym_(out, 256, 256, kernel, ksize); return; }
      if( h == 128 && w ==  128 ) { convolve_sym_(out, 128, 128, kernel, ksize); return; }
      if( h == 128 && w ==  192 ) { convolve_sym_(out, 128, 192, kernel, ksize); return; }
//       cout<<"[convolve_sym] insert this h,w to unrolling list: "<<h<<" "<<w<<endl;
      convolve_sym_(out, h, w, kernel, ksize);
   }
   inline void convolve_sym( double* image, int h, int w, double* kernel, int ksize, double* out=NULL )
   {
      if( out == NULL ) out = image;
      else memcpy( out, image, sizeof(double)*h*w );

      if( h == 240 && w ==  320 ) { convolve_sym_(out, 240, 320, kernel, ksize); return; }
      if( h == 480 && w ==  640 ) { convolve_sym_(out, 480, 640, kernel, ksize); return; }
      if( h == 512 && w ==  512 ) { convolve_sym_(out, 512, 512, kernel, ksize); return; }
      if( h == 512 && w ==  768 ) { convolve_sym_(out, 512, 768, kernel, ksize); return; }
      if( h == 600 && w ==  800 ) { convolve_sym_(out, 600, 800, kernel, ksize); return; }
      if( h == 768 && w == 1024 ) { convolve_sym_(out, 768, 1024, kernel, ksize); return; }
      if( h == 1024 && w == 768 ) { convolve_sym_(out, 1024, 768, kernel, ksize); return; }
      if( h == 256 && w ==  256 ) { convolve_sym_(out, 256, 256, kernel, ksize); return; }
      if( h == 128 && w ==  128 ) { convolve_sym_(out, 128, 128, kernel, ksize); return; }
      if( h == 128 && w ==  192 ) { convolve_sym_(out, 128, 192, kernel, ksize); return; }
//       cout<<"[convolve_sym] insert this h,w to unrolling list: "<<h<<" "<<w<<endl;
      convolve_sym_(out, h, w, kernel, ksize);
   }
}
#endif
