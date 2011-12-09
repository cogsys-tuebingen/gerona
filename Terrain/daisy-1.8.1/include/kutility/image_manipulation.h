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

#ifndef KUTILITY_IMAGE_MANIPULATION_H
#define KUTILITY_IMAGE_MANIPULATION_H

#include "kutility/kutility.def"
#include "kutility/general.h"

namespace kutility
{
   template<typename T1, typename T2>
   void scale( T1* src, int h, int w, float sc, T2* dst, int dh, int dw )
   {
      int nh = int( h*sc );
      int nw = int( w*sc );

      assert( dst != NULL );
      assert( nh == dh );
      assert( nw == dw );

      if( sc == 1 )
      {
         for( int i=0; i<h*w; i++ )
            dst[i] = (T2)src[i];
         return;
      }

      double scale_factor = 1.0 / sc;
      memset(dst, 0, sizeof(T2)*dh*dw );
      float y,x;
      for( int ny=0; ny<nh; ny++ )
      {
         y = ny * scale_factor;
         if( y>= h-1 ) continue;
         for( int nx=0; nx<nw; nx++ )
         {
            x = nx * scale_factor;
            if( x>= w-1 ) continue;
            dst[ny*nw+nx] = (T2)bilinear_interpolation(src, w, x, y);
         }
      }
   }

   template<class T> inline
   void rgb_to_y(T* cim, int h, int w, T* gim )
   {
      assert( (gim!=NULL) && (cim!=NULL) );

      for( int y=0; y<h; y++ )
      {
         for( int x=0; x<w; x++ )
         {
            int index=y*w+x;

            float r=cim[3*index  ];
            float g=cim[3*index+1];
            float b=cim[3*index+2];

            gim[index] = T( 0.299*r + 0.587*g + 0.114*b );
         }
      }
   }

   template<class T> inline
   void y_to_rgb(T* yim, int h, int w, T* rgbim )
   {
      assert( rgbim != NULL );

      int wh = w*h;

      for( int k=0; k<wh; k++ )
      {
         rgbim[ 3*k   ] = yim[k];
         rgbim[ 3*k+1 ] = yim[k];
         rgbim[ 3*k+2 ] = yim[k];
      }
   }

   template<class T> inline
   void rgb_to_bgr(T* rgb, int h, int w, T* bgr )
   {
      assert( bgr != NULL );
      int wh3 = w*h*3;

      for( int k=0; k<wh3; k+=3 )
      {
         T tmp = bgr[k];
         rgb[ k   ] = bgr[ k+2 ];
         rgb[ k+1 ] = bgr[ k+1 ];
         rgb[ k+2 ] = tmp;
      }
   }

   template<class T> inline
   void bgr_to_rgb(T* bgr, int h, int w, T* rgb )
   {
      rgb_to_bgr(bgr,h,w,rgb);
   }

   template<class T> inline void rgba_to_y(T* cim, int h, int w, T* gim )
   {
      assert( (gim!=NULL) && (cim!=NULL) );

      for( int y=0; y<h; y++ )
      {
         for( int x=0; x<w; x++ )
         {
            int index=y*w+x;

            float r=cim[4*index  ];
            float g=cim[4*index+1];
            float b=cim[4*index+2];

            gim[index] = T( 0.299*r + 0.587*g + 0.114*b );
         }
      }
   }
   template<class T> inline void rgba_to_rgb(T* rgbaim, int h, int w, T* rgbim )
   {
      assert( (rgbim!=NULL) && (rgbaim!=NULL) );
      int wh = w*h;
      for( int k=0; k<wh; k++ )
      {
         rgbim[3*k  ] = rgbaim[4*k  ];
         rgbim[3*k+1] = rgbaim[4*k+1];
         rgbim[3*k+2] = rgbaim[4*k+2];
      }
   }

   uchar* clean_image   (uchar * &image, int w, int h, bool in_place=false);
   uchar* apply_erosion (uchar * &image, int w, int h, bool in_place=false);
   uchar* apply_dilation(uchar * &image, int w, int h, bool in_place=false);
   uchar* down_sample   (uchar *  image, int w, int h);

   uchar* resize_image( uchar* &image, int h, int w, int nh, int nw, bool in_place=false);

   /// scales the image intensity between a lower "il" and an upper
   /// "iu" value. "sz" is the image size.
   /// by deafult il=0 and ui = 1;
   double* scale_intensity( uchar* image, int sz, double il=0, double iu=1);

   template<class T>
   void decompose_channels( T* image, int h, int w, T* &ch_0, T* &ch_1, T* &ch_2)
   {
      int image_size = h*w;

      ch_0 = kutility::allocate<uchar>(image_size);
      ch_1 = kutility::allocate<uchar>(image_size);
      ch_2 = kutility::allocate<uchar>(image_size);

#if defined(WITH_OPENMP)
#pragma omp parallel for
#endif
      for( int y=0; y<h; y++ )
      {
         int yw = y*w;
         for( int  x=0; x<w; x++ )
         {
            int index = yw+x;
            int cindex = 3*index;

            ch_0[index] = image[index  ];
            ch_1[index] = image[index+1];
            ch_2[index] = image[index+2];
         }
      }
   }

   /// applies gamma correction
   template<class T> inline
   T* gamma_correction( T* im, int h, int w, double gamma, bool in_place=false)
   {
      int sz = w*h;
      T* out;

      if( !in_place )
         out = kutility::allocate<T>(sz);
      else
         out = im;

      double val;

      for( int i=0; i<sz; i++ )
      {
         val = (pow( (double)im[i], gamma ));
         if( val > 255 )
            out[i] = (T)255;
         else
            out[i] = (T)val;
      }
      return out;
   }

   /// adds some noise to the pixels
   template<class T> inline
   T* add_noise( T* im, int h, int w, int noise_level, bool in_place=false)
   {
      int sz = w*h;
      T* out;

      if( !in_place )
         out = kutility::allocate<T>(sz);
      else
         out = im;


      for( int i=0; i<sz; i++ )
      {
         int sign = 1;
         if( rand()/(double)RAND_MAX < 0.5 ) sign = -1;

         out[i] = im[i] + sign * rand()/(double)RAND_MAX * noise_level;
      }
      return out;
   }
}
#endif
