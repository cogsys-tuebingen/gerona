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

#include <kutility/image_manipulation.h>
#include <kutility/math.h>

namespace kutility
{
   double* scale_intensity( uchar* image, int sz, double il, double iu)
   {
      double* out = kutility::allocate<double>(sz);

      double mult = (iu-il)/255.0;

      for( int i=0; i<sz; i++ )
      {
         out[i] = ((double)image[i]) * mult + il;
      }

      return out;
   }

   /// resizes the image to nh x nw using bilinear_interpolation
   uchar* resize_image( uchar* &image, int h, int w, int nh, int nw, bool in_place )
   {
      uchar* out = kutility::zeros<uchar>( nh*nw );

      double ratioy = h / (double)nh;
      double ratiox = w / (double)nw;

      int y, x, ynw;
      double ny, nx;

      #pragma omp parallel for private( y, x, ny, nx, ynw )
      for( y=0; y<nh; y++ )
      {
         ny = y * ratioy;

         ynw = y * nw;

         for( x=0; x<nw; x++ )
         {
            nx = x * ratiox;

            out[ ynw + x ] = (uchar)bilinear_interpolation( image, w, nx, ny );
         }
      }

      if( in_place )
      {
         deallocate( image );
         image = out;
      }

      return out;
   }

   /// image must be binary {0,1}
   uchar* clean_image(uchar * &image, int w, int h, bool in_place)
   {
      if( in_place )
      {
         apply_erosion ( image, w, h, in_place);
         apply_dilation( image, w, h, in_place);
         return image;
      }
      else
      {
         uchar* tmp_image = apply_erosion ( image, w, h, in_place);
         uchar* output    = apply_dilation( tmp_image, w, h, in_place);

         delete []tmp_image;
         return output;
      }
   }

   uchar* apply_dilation(uchar * &image, int w, int h, bool in_place)
   {
      int i,j;
      int index;
      int sz  = h*w;

      uchar * output = new uchar[sz];
      memset(output, 0, sizeof(uchar)*sz );

      for( i=1; i<h-1; i++ )
      {
         for( j=1; j<w-1; j++ )
         {
            index = i*w+j;
            if( image[index]==1 )
            {
               output[index-1-w] = image[index-1-w] || output[index-1-w];
               output[index+1-w] = image[index+1-w] || output[index+1-w];
               output[index-1+w] = image[index-1+w] || output[index-1+w];
               output[index+1+w] = image[index+1+w] || output[index+1+w];
               output[index-w]   = 1;
               output[index+w]   = 1;
               output[index-1]   = 1;
               output[index+1]   = 1;
            }
         }
      }

      if( in_place )
      {
         delete []image;
         image = output;
         return image;
      }
      else
      {
         return output;
      }
   }

   uchar* apply_erosion(uchar * &image, int w, int h, bool in_place)
   {
      int i,j;
      int sum=0;
      int index;
      int frameSize  = h*w;

      uchar * output = new uchar[frameSize];
      memset(output, 0, sizeof(uchar)*frameSize );

      int seed_threshold = 5; // seed is assumed to be [0 1 0; 1 1 1; 0 1 0]; sum is 5;

      for( i=1; i<h-1; i++ )
      {
         for( j=1; j<w-1; j++ )
         {
            index = i*w+j;
            sum=0;
            if( image[index  ] == 1 ) sum++;
            if( image[index-w] == 1 ) sum++;
            if( image[index+w] == 1 ) sum++;
            if( image[index-1] == 1 ) sum++;
            if( image[index+1] == 1 ) sum++;

            if( sum == seed_threshold )
               output[index]=1;
            else
               output[index]=0;
         }
      }

      if( in_place )
      {
         delete []image;
         image = output;
         return image;
      }
      else
      {
         return output;
      }
   }

   uchar* down_sample(uchar* image, int w, int h)
   {
      int w_s = w>>1;
      int h_s = h>>1;

      int tmpIndex1, tmpIndex2, tmpIndex3, tmpIndex4, tmpIndex5;

      uchar * out = new uchar[3*w_s*h_s];
      int i;

      for( i=0; i<h_s; i++ )
      {
         for(int j=0; j<w_s; j++)
         {
            tmpIndex1 = 3*(i*w_s+j);
            tmpIndex2 = 6*(i*w  +j);

            tmpIndex3 = tmpIndex2 + 3;
            tmpIndex4 = tmpIndex2 + 3*w;
            tmpIndex5 = tmpIndex2 + 3*w+3;

            out[tmpIndex1  ] = ( image[tmpIndex2  ] + image[tmpIndex3  ] + image[tmpIndex4  ] + image[tmpIndex5  ] ) >> 2;
            out[tmpIndex1+1] = ( image[tmpIndex2+1] + image[tmpIndex3+1] + image[tmpIndex4+1] + image[tmpIndex5+1] ) >> 2;
            out[tmpIndex1+2] = ( image[tmpIndex2+2] + image[tmpIndex3+2] + image[tmpIndex4+2] + image[tmpIndex5+2] ) >> 2;
         }
      }
      return out;
   }

   int threshold_yen( double *array, int sz)
   {
      int    i,c,c2;			//counters
      double rho=0.15,threshold=0;
      double h=0;
      double hf,hb;		       	//total objective fn, foreground and background parts
      double currentMaxH=0;		//threshold and current max total entropy
      double scale=1.0/(1-rho);		//used in calculation of entropic correlation

      double pC=0;			//cumulative probabilities
      double pmf[256];			//probability mass function

//		for( i=0; i<256; i++ ) pmf[i]=0;
      memset(pmf, 0, sizeof(double)*256 );

      //calculation of pmf
      for(i=0; i<256; i++) pmf[i]=array[i]/sz;

      for( c=0; c<256; c++ )
      {

         if( pmf[c] != 0 )
         {
            pC += pmf[c];		//calculate cumulative probabilities

            //initialization
            hf=0;
            hb=0;

            //foreground part
            for( c2=0; c2<c;   c2++ ) if( pmf[c2] >= 0 ) hf += pow( pmf[c2] , rho );

            //background part
            for( c2=c; c2<256; c2++ ) if( pmf[c2] >  0 ) hb += pow( pmf[c2] , rho );

            //total objective function
            if( pC < 0.99999999999 ) h = scale*(log(hf*hb)-rho*log(pC*(1-pC)));

            //check if max
            if( h>currentMaxH ) { threshold = c; currentMaxH = h; }
         }
      }

      return (int)threshold;
   }


}

