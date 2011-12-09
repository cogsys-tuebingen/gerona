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

#ifndef KUTILITY_MATH_H
#define KUTILITY_MATH_H

#include "kutility/general.h"
// #include "kutility/linear_algebra.h"
#include "kutility/convolution.h"

namespace kutility
{
   template<typename T>
    inline T distance( T a[2], T b[2] )
    {
        T d0 = a[0]-b[0];
        T d1 = a[1]-b[1];
        return sqrt( d0*d0+d1*d1 );
    }

   template<typename T> inline
   void shift_array_right( T* arr, int sz, int start )
   {
      for( int i=sz-2; i>=start; i-- )
      {
         arr[i+1] = arr[i];
      }
   }

   /// creates a 1D gaussian filter with N(mean,sigma).
   inline void gaussian_1d(float* fltr, int fsz, float sigma, float mean )
   {
      assert(fltr != NULL);
      int sz = (fsz-1)/2;
      int counter=-1;
      float sum = 0.0;
      float v = 2*sigma*sigma;
      for( int x=-sz; x<=sz; x++ )
      {
         counter++;
         fltr[counter] = exp((-(x-mean)*(x-mean))/v);
         sum += fltr[counter];
      }

      if( sum != 0 )
      {
         for( int x=0; x<fsz; x++ )
            fltr[x] /= sum;
      }
   }

   /// creates a 2D gaussian filter with N(mean,sigma).
   inline float* gaussian_2d(int fsz, float sigma, float mn)
   {
      int fltr_size = fsz * fsz;
      float* fltr = new float[fltr_size];

      int sz = (fsz-1)/2;

      int y,x;
      int counter=-1;
      float sum=0;

      float v = 2*sigma*sigma;

      for( y=-sz; y<=sz; y++ )
      {
         for( x=-sz; x<=sz; x++ )
         {
            counter++;
            fltr[counter] = exp((-(x)*(x-mn)-(y-mn)*(y-mn))/v);
            sum += fltr[counter];
         }
      }

      if( sum != 0 )
      {
         for( x=0; x<fltr_size; x++ )
            fltr[x] /= sum;
      }

      return fltr;
   }

   template<class T1, class T2> inline
   double normalized_cross_correlation( T1* a, T2* b, int sz )
   {
      double mean_a = 0;
      double mean_b = 0;
      for( int i=0; i<sz; i++ )
      {
         mean_a += a[i];
         mean_b += b[i];
      }
      mean_a /= sz;
      mean_b /= sz;

      double var_a = 0;
      double var_b = 0;
      double var_ab = 0;

      double a_part = 0;
      double b_part = 0;

      for( int i=0; i<sz; i++ )
      {
         a_part = ( a[i] - mean_a );
         b_part = ( b[i] - mean_b );

         var_a  += a_part * a_part;
         var_b  += b_part * b_part;
         var_ab += a_part * b_part;
      }

      var_a /= sz;
      var_b /= sz;
      var_ab /= sz;

      if( var_a != 0 && var_b != 0 )
         return var_ab / sqrt(var_a * var_b);
      else if ( var_a == 0 && var_b == 0 )
         return 1;
      else
         return -1;
   }

   inline float pi() { return atan2( 0.0f, -1.0f ); }

   /// Applies a 2d gaussian blur of sigma std to the input array.  if
   /// kernel_size is not set or it is set to 0, then it is taken as
   /// 3*sigma and if it is set to an even number, it is incremented
   /// to be an odd number.  if in_place=true, then T1 must be equal
   /// to T2 naturally.
   template<class T1, class T2> inline
   T1* blur_gaussian_2d( T2* array, int rn, int cn, float sigma, int kernel_size=0, bool in_place=false )
   {
      T1* out = NULL;

      if( in_place )
         out = (T1*)array;
      else
         out = type_cast<T1,T2>(array,rn*cn);

      if( kernel_size == 0 )
         kernel_size = (int)(3*sigma);

      if( kernel_size%2 == 0 ) kernel_size++; // kernel size must be odd
      if( kernel_size < 3 ) kernel_size= 3;  // kernel size cannot be smaller than 3

      float* kernel = new float[kernel_size];
      gaussian_1d(kernel, kernel_size, sigma, 0);

      // !! apply the filter separately
      convolve_sym( out, rn, cn, kernel, kernel_size );
      // conv_horizontal( out, rn, cn, kernel, kernel_size);
      // conv_vertical  ( out, rn, cn, kernel, kernel_size);

      deallocate(kernel);
      return out;
   }

   /// inserts a portion of the source to the destination.
   template<class Td, class Ts>
   void insert( Td* dst, int dcn, int dymin, int dymax, int dxmin, int dxmax,
                Ts* src, int scn, int symin=-1, int symax=-1, int sxmin=-1, int sxmax=-1 )
   {
      int xsz = dxmax - dxmin;
      int ysz = dymax - dymin;

      if( symin == -1 &&  symax == -1 &&  sxmin == -1 &&  sxmax == -1 )
      {
         sxmin = 0;
         symin = 0;
         sxmax = scn;
         symax = ysz;
      }

      if( ysz != symax - symin ) error("insert: intervals must match");
      if( xsz != sxmax - sxmin ) error("insert: intervals must match");

      for( int y=0; y<ysz; y++ )
      {
         for( int x=0; x<xsz; x++ )
         {
            dst[ (dymin+y)*dcn+(dxmin+x) ] = (Td)( src[ (symin+y)*scn+(sxmin+x) ] );
         }
      }
   }

   /// swaps the values of y and x
   template<class T> inline
   void swap(T &y, T &x)
   {
      T tmp = x;
      x = y;
      y = tmp;
   }

   /// inverts a boolean array: 1->0 & 0->1.
   inline bool* invert( bool* data, int sz, bool in_place=true)
   {
      bool* out=NULL;
      if( in_place ) out = data;
      else           out = new bool[sz];

      for(int i=0; i<sz; i++)
         out[i] = !data[i];

      return out;
   }

   /// extracts a square patch of patch_width x patch_width from a the
   /// image around the point ry,rx ;
   /// returns true if all the pixels are within the image and false
   /// if some of the pixels are outside the image.
   template<class T1, class T2>
   bool extract_patch( T1* dst, T1* src, int h, int w, T2 ry, T2 rx, int patch_width )
   {
      float w_2 = patch_width/2;

      float x,y;
      float yy, xx;

      bool out_of_image = true;

      int index=0;

      for( y=0; y<patch_width; y++ )
      {
         for( x=0; x<patch_width; x++ )
         {
            yy = ry + y - w_2;
            xx = rx + x - w_2;

            if( is_outside( xx, 0, w, yy, 0, h ) )
            {
               dst[index] = 0;
               out_of_image = false;
            }
            else
            {
               dst[ index ] = (T1)bilinear_interpolation( src, w, xx, yy );
            }
            index++;
         }
      }

      return out_of_image;
   }

   /// extracts a square patch of patch_width x patch_width from a
   /// rotated image around the point ry,rx ; rotation_angle is in
   /// radians.  returns true if all the pixels are within the image
   /// and false if some of the pixels are outside the image.
   template<class T1, class T2, class T3>
   bool extract_rotated_patch( T1* dst, T1* src, int h, int w, T2 ry, T2 rx, int patch_width, T3 rotation_angle )
   {
      int w_2 = patch_width/2;

      float kos = cos( rotation_angle );
      float zin = sin( rotation_angle );

      int yp, xp;
      float yu, xu;
      float y, x;

      int index = 0;
      bool out_of_image = true;

      for( yp=0; yp<patch_width; yp++ )
      {
         for( xp=0; xp<patch_width; xp++ )
         {
            xu = xp-w_2;
            yu = yp-w_2;

            x  = kos * xu - zin * yu + rx;
            y  = zin * xu + kos * yu + ry;

            if( is_inside( x, 0, w, y, 0, h ) )
            {
               dst[ index ] = (T1)bilinear_interpolation(src, w, x, y);
            }
            else
            {
               dst[ index ] = 0;
               out_of_image = false;
            }
            index++;
         }
      }

      return out_of_image;
   }

   /// extracts a portion of the matrix [ymin:ymax) & [xmin:xmax)
   /// and returns the result.
   /// note: you should deallocate the dst memory yourself
   /// note: upper boundaries are not included in the output matrix
   template<class T>
   T* extract( T* src, int xmin, int xmax, int ymin, int ymax, int matw)
   {
      int xsz = xmax - xmin;
      int ysz = ymax - ymin;

      T* dst = new T[ysz*xsz];

      int yy=0;
      int xx=0;

      for( int y=ymin; y<ymax; y++ )
      {
         xx=0;
         for( int x=xmin; x<xmax; x++ )
         {
            dst[yy*xsz+xx] = src[y*matw+x];
            xx++;
         }
         yy++;
      }
      return dst;
   }

   /// extracts a portion of the matrix [ymin:ymax) & [xmin:xmax)
   /// and returns the result.
   /// note: you should deallocate the dst memory yourself
   /// note: upper boundaries are not included in the output matrix
   template<class T>
   T* crop( T* src, int h, int w, int center_y, int center_x, int patch_rad)
   {
      int pw = 2*patch_rad+1;
      int psz = pw*pw;

      T* dst = new T[psz];
      initialize( dst, psz, 0);

      int yy, xx;

      for( yy = -patch_rad; yy<=patch_rad; yy++ )
      {
         if( yy + center_y < 0 || yy + center_y > h )
            continue;

         for( xx = -patch_rad; xx<=patch_rad; xx++ )
         {
            if( xx+center_x < 0 || xx+center_x > w )
               continue;

            dst[ ( yy+patch_rad ) * pw + xx + patch_rad ] = src[ (yy+center_y)*w+xx+center_x ];
         }
      }

      return dst;
   }

   /// extracts a portion of the matrix [ymin:ymax) & [xmin:xmax)
   /// and returns the result in the given pointer.
   /// note: you should allocate the dst memory yourself
   /// note: upper boundaries are not included in the output matrix
   template<class T>
   void extract( T* dst, T* src, int xmin, int xmax, int ymin, int ymax, int mw)
   {
      int xsz = xmax - xmin;

      int yy=0;
      int xx=0;

      for( int y=ymin; y<ymax; y++ )
      {
         xx=0;
         for( int x=xmin; x<xmax; x++ )
         {
            dst[yy*xsz+xx] = src[y*mw+x];
            xx++;
         }
         yy++;
      }
   }

   /// extracts a portion of the matrix [ymin:ymax) & [xmin:xmax)
   /// and returns the result in the given pointer.
   /// note: you should deallocate the dst memory yourself
   /// note: upper boundaries are not included in the output matrix
   template<class T>
   T** extract( T** src, int xmin, int xmax, int ymin, int ymax)
   {
      int xsz = xmax - xmin;
      int ysz = ymax - ymin;

      T** dst = allocate<T>(ysz,xsz);

      int yy=0;
      int xx=0;

      for( int y=ymin; y<ymax; y++ )
      {
         xx=0;
         for( int x=xmin; x<xmax; x++ )
         {
            dst[yy][xx] = src[y][x];
            xx++;
         }
         yy++;
      }
      return dst;
   }

   /// extracts a portion of the matrix [ymin:ymax) & [xmin:xmax)
   /// and returns the result in the given pointer.
   /// note: you should allocate the dst memory yourself
   /// note: upper boundaries are not included in the output matrix
   template<class T>
   void extract( T** dst, T** src, int xmin, int xmax, int ymin, int ymax)
   {
      int yy=0;
      int xx=0;

      for( int y=ymin; y<ymax; y++ )
      {
         xx=0;
         for( int x=xmin; x<xmax; x++ )
         {
            dst[yy][xx] = src[y][x];
            xx++;
         }
         yy++;
      }
   }

   /// rounds a number: if real part is bigger than 0,5 rounds up else down
   template<class T> inline
   T round( T x )
   {
      T fx = floor(x);
      if( x-fx > 0.5 ) return fx+1;
      else             return fx;
   }

   /// rounds an array of numbers: if real part is bigger than 0,5
   /// rounds up else down.
   template<class T> inline
   T* round( T* x, int sz, bool in_place = false )
   {
      T* out;
      if( in_place ) out = x;
      else           out = allocate<T>(sz);

      for( int i=0; i<sz; i++ )
      {
         out[i] = round(x[i]);
      }
      return out;
   }

//    /// filters the image with a filter.
//    float* filter_2d(float* &im, int r, int c, float* filter, int fr, int fc, bool in_place=false);
   /// r & c is the size of the image and filter has fr, fc size. it
   /// supports in place filtering. it uses simple for loops and does
   /// not employs a fast convolution implementation. beware: it is
   /// not equal to convolution - it does not invert the filter.
   template<class T>
   T* filter_2d(T* &im, int r, int c, T* filter, int fr, int fc, bool in_place)
   {
      int y,x;
      int yy,xx;

      int ya,xa;

      int yc, yac;

      int fr_half = fr/2;
      int fc_half = fc/2;

      int sz = r*c;

      T* out = allocate<T>(sz);
      initialize(out, sz, 0);

      T sum;
      int findex=0;

      for( y=0; y<r; y++ )
      {
         ya = y - fr_half-1;
         yc = y*c;

         for( x=0; x<c; x++ )
         {
            sum = 0;
            xa = x-fc_half-1;
            findex=0;
            for( yy=0; yy<fr; yy++ )
            {
               ya++;
               if( is_outside(ya, 0, r) )
               {
                  findex += fc;
                  continue;
               }
               yac = ya*c;

               for( xx=0; xx<fc; xx++ )
               {
                  xa++;
                  if( is_outside(xa,0,c) )
                  {
                     findex++;
                     continue;
                  }
                  sum += im[yac+xa]*filter[findex++];
               }
               xa -= fc;
            }
            ya -= fr;
            out[yc++] = sum;
         }
      }

      if( in_place )
      {
         delete []im;
         im = out;
      }

      return out;
   }

   /// returns an array filled with ones.
   template<class T> inline
   T* ones (int r)
   {
      T* data = allocate<T>(r);
      for( int i=0; i<r; i++ )
         data[i] = 1;
      return data;
   }

   /// returns an array filled with zeroes.
   template<class T> inline
   T* zeros(int r)
   {
      T* data = allocate<T>(r);
      memset( data, 0, sizeof(T)*r );
      return data;
   }

   /// computes the square of a number and returns it.
   template<class T> inline
   T square(T a)
   {
      return a*a;
   }

   /// computes the square of an array. if in_place is enabled, the
   /// result is returned in the array arr.
   template<class T> inline
   T* square(T* arr, int sz, bool in_place=false)
   {
      T* out;
      if( in_place ) out = arr;
      else           out = allocate<T>(sz);

      for( int i=0; i<sz; i++ )
         out[i] = arr[i]*arr[i];

      return out;
   }

   /// computes the p power of a number and returns it.
   template<class T1, class T2> inline
   T1 power(T1 a, T2 p)
   {
      return (T1)pow(a,p);
   }

   /// computes the p power of an array. if in_place is enabled, the
   /// result is returned in the array arr.
   template<class T1, class T2> inline
   T1* power(T1* arr, int sz, T2 p, bool in_place=false)
   {
      T1* out;
      if( in_place ) out = arr;
      else           out = allocate<T1>(sz);

      for( int i=0; i<sz; i++ )
         out[i] = power(arr[i],p);

      return out;
   }

   /// returns the theta component of a point in the range -PI to PI.
   template<class T> inline
   float angle(T x, T y)
   {
      return atan2( (float)y, (float)x );
   }

   /// returns the theta component of a point array in the range -PI to PI.
   template<class T> inline
   float* angle(T* x, T* y, int lsz)
   {
      float* ang = allocate<float>(lsz);

      for( int k=0; k<lsz; k++ )
      {
         ang[k] = angle<T>(x[k],y[k]);
      }

      return ang;
   }

   /// returns the radial component of a point.
   template<class T> inline
   T magnitude(T x, T y)
   {
      return sqrt(x*x+y*y);
   }

   /// computes the radial component of a 2D array and returns the
   /// result in a REAL array. the x&y coordinates are given in
   /// separate 1D arrays together with their size.
   template<class T> inline
   T* magnitude(T* arrx, T* arry, int lsz)
   {
      T* mag = allocate<T>(lsz);

      for( int k=0; k<lsz; k++ )
      {
         mag[k] = sqrt( arrx[k]*arrx[k] + arry[k]*arry[k] );
      }

      return mag;
   }

   /// Converts the given cartesian coordinates of a point to polar
   /// ones.
   template<class T> inline
   void cartesian2polar(T x, T y, float &r, float &th)
   {
      r  = magnitude(x,y);
      th = angle(x,y);
   }

   /// Converts the given polar coordinates of a point to cartesian
   /// ones.
   template<class T1, class T2> inline
   void polar2cartesian(T1 r, T1 t, T2 &y, T2 &x)
   {
      x = (T2)( r * cos( t ) );
      y = (T2)( r * sin( t ) );
   }

   /// returns an interval list that starts at "st" and ends at "en"
   /// having "level_no" levels. The list has entries like :
   /// [ s1 e1 ;
   ///   s2 e2 ;
   ///   ....
   ///   sn en ] -> s(i+1) = e(i)
   /// the function uses upto 4 point precisions if not specified
   template<class T> inline
   T** interval( T st, T en, int levels, int prec=4)
   {
      T** interval_list = allocate<T>(levels, 2);

      float step = ((float)(en-st))/levels;

      for( int i=0; i<levels; i++ )
      {
         interval_list[i][0] = i*step+st;
         interval_list[i][1] = i*step+st+step;
      }
      return interval_list;
   }

   /// computes the gradient of an image and returns the result in
   /// pointers to REAL.
   template <class T> inline
   void gradient(T* im, int h, int w, T* dy, T* dx)
   {
      assert( dx != NULL );
      assert( dy != NULL );

      for( int y=0; y<h; y++ )
      {
         int yw = y*w;
         for( int x=0; x<w; x++ )
         {
            int ind = yw+x;
            // dx
            if( x>0 && x<w-1 ) dx[ind] = ((T)im[ind+1]-(T)im[ind-1])/2.0;
            if( x==0         ) dx[ind] = ((T)im[ind+1]-(T)im[ind]);
            if( x==w-1       ) dx[ind] = ((T)im[ind  ]-(T)im[ind-1]);

            //dy
            if( y>0 && y<h-1 ) dy[ind] = ((T)im[ind+w]-(T)im[ind-w])/2.0;
            if( y==0         ) dy[ind] = ((T)im[ind+w]-(T)im[ind]);
            if( y==h-1       ) dy[ind] = ((T)im[ind]  -(T)im[ind-w]);
         }
      }
   }

   template<class T> inline
   T is_positive( T number )
   {
      if( number > 0 ) return number;
      else return (T)(0);
   }

   template<class T> inline
   T* layered_gradient( T* data, int h, int w, int layer_no=8 )
   {
      int data_size = h * w;
      T* layers = zeros<T>(layer_no * data_size);

      // smooth the data matrix
      T* bdata = blur_gaussian_2d<T,T>( data, h, w, 0.5, 5, false);

      T *dx = new T[data_size];
      T *dy = new T[data_size];
      gradient(bdata, h, w, dy, dx);
      deallocate( bdata );

#if defined(WITH_OPENMP)
#pragma omp parallel for
#endif
      for( int l=0; l<layer_no; l++ )
      {
         float angle = 2*l*pi()/layer_no;
         float kos = cos( angle );
         float zin = sin( angle );

         T* layer_l = layers + l*data_size;

         for( int index=0; index<data_size; index++ )
         {
            float value = kos * dx[ index ] + zin * dy[ index ];
            if( value > 0 ) layer_l[index] = value;
            else            layer_l[index] = 0;
         }
      }
      deallocate(dy);
      deallocate(dx);

      return layers;
   }

   /// be careful, 'data' is destroyed afterwards
   template<class T> inline
   void layered_gradient( T* data, int h, int w, int layer_no, T* layers, T* workspace=0, int lwork=0 )
   {
      int data_size = h * w;
      assert(layers!=NULL);
      memset(layers,0,sizeof(T)*data_size*layer_no);

      bool empty=false;
      T* work=NULL;
      if( lwork < 3*data_size ) {
         work = new T[3*data_size];
         empty=true;
      }

      // // smooth the data matrix
      // T* bdata = blur_gaussian_2d<T,T>( data, h, w, 0.5, 5, false);
      float kernel[5]; gaussian_1d(kernel, 5, 0.5, 0);
      memcpy( work, data, sizeof(T)*data_size);
      convolve_sym( work, h, w, kernel, 5 );

      T *dx = work+data_size;
      T *dy = work+2*data_size;
      gradient( work, h, w, dy, dx );

#if defined(WITH_OPENMP)
#pragma omp parallel for
#endif
      for( int l=0; l<layer_no; l++ )
      {
         float angle = 2*l*pi()/layer_no;
         float kos = cos( angle );
         float zin = sin( angle );

         T* layer_l = layers + l*data_size;

         for( int index=0; index<data_size; index++ )
         {
            float value = kos * dx[ index ] + zin * dy[ index ];
            if( value > 0 ) layer_l[index] = value;
            else            layer_l[index] = 0;
         }
      }
      if( empty ) delete []work;
   }


   /// computes the bilinearly interpolated value of the point (x,y).
   template<class T1, class T2> inline
   float bilinear_interpolation(T1* arr, int w, T2 x, T2 y)
   {
      int mnx = (int)floor( x );
      int mny = (int)floor( y );
      int mxx = (int) ceil( x );
      int mxy = (int) ceil( y );

      double alfa = mxx - x;
      double beta = mxy - y;

      if( alfa < 0.001 ) alfa = 0;
      if( beta < 0.001 ) beta = 0;

      int mnyw = mny * w;
      int mxyw = mxy * w;

      if( alfa < 0.001 ) return float(beta * arr[mnyw+mxx] + (1-beta) * arr[mxyw+mxx]);
      if( alfa > 0.999 ) return float(beta * arr[mnyw+mnx] + (1-beta) * arr[mxyw+mnx]);
      if( beta < 0.001 ) return float(alfa * arr[mxyw+mnx] + (1-alfa) * arr[mxyw+mxx]);
      if( beta > 0.999 ) return float(alfa * arr[mnyw+mnx] + (1-alfa) * arr[mnyw+mxx]);

      return float( beta*(alfa * arr[mnyw+mnx] + (1-alfa)*arr[mnyw+mxx] )
                   +(1-beta)*(alfa * arr[mxyw+mnx] + (1-alfa)*arr[mxyw+mxx] ) );
   }

   /// divides the elements of the array with "norm". function
   /// supports in-place operations in which case the result is casted
   /// to the input type; default is non-in-place.
   template<class T1, class T2> inline
   T2* normalize(T1* data, int sz, T2 norm, bool in_place=false)
   {
      assert( norm != 0.0 );

      float inv_norm = 1.0/norm;
      if( in_place )
      {
         for( int i=0; i<sz; i++ )
         {
            data[i] = (T1)(data[i]*inv_norm);
         }
         return NULL;
      }
      else
      {
         T2* new_data = allocate<T2>(sz);

         for( int i=0; i<sz; i++ )
         {
            new_data[i] = (T2)(data[i]*inv_norm);
         }
         return new_data;
      }
   }

   template<typename T> inline
   void diff( const T* a, const T* b, const int sz, T* a_m_b)
   {
      for( int k=0; k<sz; k++ )
         a_m_b[k] = a[k] - b[k];
   }

   /// computes the difference of two arrays and returns the resulting
   /// array. function supports in place operation, and returns the
   /// result in the "a" array if in place is enabled.
   template<class T> inline
   T* diff( T* a, const T* b, const int sz, bool in_place=false)
   {
      T* d=NULL;
      if( in_place ) d = a;
      else           d = allocate<T>(sz);

      for( int k=0; k<sz; k++ )
      {
         d[k] = a[k]-b[k];
      }
      return d;
   }

   /// computes the absolute difference of two arrays and returns the
   /// resulting array : d = |a-b|. function supports in place
   /// operation, and returns the result in the "a" array if in place
   /// is enabled.
   template<class T> inline
   T* absdiff( T* a, T* b, int sz, bool in_place=false)
   {
      T* d=NULL;
      if( in_place ) d = a;
      else           d = allocate<T>(sz);

      for( int k=0; k<sz; k++ )
      {
         d[k] = (T)fabs(a[k]-b[k]);
      }
      return d;
   }

   /// computes the absolute difference of two matrices and returns
   /// the resulting matrix : d = |a-b|. function supports in place
   /// operation, and returns the result in the "a" matrix if in place
   /// is enabled.
   template<class T> inline
   T** absdiff( T** a, T** b, int ysz, int xsz, bool in_place=false)
   {
      T** d=NULL;
      if( in_place ) d = a;
      else           d = allocate<T>(ysz,xsz);

      for( int y=0; y<ysz; y++ )
      {
         for( int x=0; x<xsz; x++ )
         {
            d[y][x] = fabs(a[y][x]-b[y][x]);
         }
      }
      return d;
   }


   /// computes the l1norm of an array: sum_i( |a(i)| )
   template<class T> inline
   T l1norm( T* a, int sz)
   {
      T norm=0;
      for( int k=0; k<sz; k++ )
         norm += abs( a[k] );
   }

   /// computes the l1norm of the difference of two arrays: sum_i( a(i)-b(i) )
   template<class T> inline
   T l1norm( T* a, T* b, int sz)
   {
      T norm=0;
      for( int k=0; k<sz; k++ )
         norm += abs(a[k]-b[k]);
      return norm;
   }

   /// computes the l2norm of an array: [ sum_i( [a(i)]^2 ) ]^0.5
   template<class T> inline
   float l2norm( T* a, int sz)
   {
      float norm=0;
      for( int k=0; k<sz; k++ )
         norm += a[k]*a[k];
      return sqrt(norm);
   }

   /// computes the l2norm of the difference of two arrays: [ sum_i( [a(i)-b(i)]^2 ) ]^0.5
   template<class T1, class T2> inline
   float l2norm( T1* a, T2* b, int sz)
   {
      float norm=0;
      for( int i=0; i<sz; i++ )
      {
         norm += square( (float)a[i] - (float)b[i] );
      }
      norm = sqrt( norm );

      return norm;
   }

   template<class T> inline
   float l2norm( T y0, T x0, T y1, T x1 )
   {
      float d0 = x0 - x1;
      float d1 = y0 - y1;

      return sqrt( d0*d0 + d1*d1 );
   }

   /// computes the l2 norm of the difference of two arrays by
   /// weighting regions of them. if reg is set to -1 (or not
   /// specified) each difference is weighted.  if reg is not -1,
   /// arrays are assumed to be composed of sz/reg segments and the
   /// weighting is applied to these segments.  reg must be a integer
   /// multiple of sz.
   template<class T1, class T2> inline
   float weighted_l2_norm(T1* a, T1* b, int sz, T2* w=NULL, int reg=-1)
   {
      if( w == NULL )
         error("weight array is NULL. use more efficient l2norm instead");

      int wsz;
      if( reg == -1 ) wsz = sz;
      else            wsz = reg;

      int rsz = sz / reg;

      if( rsz*reg != reg )
         error("reg must be an integer multiple of array size sz");

      int k;
      float norm=0;
      float sub_norm=0;
      for( k=0; k<wsz; k++ )
      {
         sub_norm = l2norm( a+k*rsz, b+k*rsz, rsz );
         norm += w[k] * sub_norm;
      }
      return norm;
   }

   template<class T1, class T2> inline
   float mean_absolute_difference( T1* arr1, T2* arr2, int size)
   {
      float mad_score=0;

      for( int i=0; i<size; i++ )
      {
         mad_score += fabs( (float)arr1[i] - (float)arr2[i] );
      }

      return mad_score/size;
   }

   /// adds a constant number to every number in the array;
   template<class T1, class T2> inline
   T1* add(T1* arr, int sz, T2 num, bool in_place=false)
   {
      T1* out;

      if( in_place ) out = arr;
      else           out = allocate<T1>(sz);

      for( int i=0; i<sz; i++ )
      {
         out[i] = arr[i] + (T1)num;
      }
      return out;
   }

   /// adds a constant number to every number in the matrix;
   template<class T1, class T2> inline
   T1** add(T1** arr, int ysz, int xsz, T2 num, bool in_place=false)
   {
      T1** out;

      if( in_place ) out = arr;
      else           out = allocate<T1>(ysz,xsz);

      for( int y=0; y<ysz; y++ )
      for( int x=0; x<xsz; x++ )
      {
         out[y][x] = arr[y][x] + (T1)num;
      }
      return out;
   }

   /// subtracts a constant number from every element in the array;
   template<class T1, class T2> inline
   T1* subt(T1* arr, int sz, T2 num, bool in_place=false)
   {
      T1* out = add(arr,sz,-num,in_place);
      return out;
   }

   /// subtracts a constant number from every element in the matrix;
   template<class T1, class T2> inline
   T1** subt(T1** arr, int ysz, int xsz, T2 num, bool in_place=false)
   {
      T1* out = add(arr,ysz,xsz,-num,in_place);
      return out;
   }

   /// divides the elements of the array with num
   template<class T1, class T2> inline
   void divide(T1* arr, int sz, T2 num )
   {
      float inv_num = 1.0 / num;

      for( int i=0; i<sz; i++ )
      {
         arr[i] = (T1)(arr[i]*inv_num);
      }
   }

   /// thresholds the data.
   template<class T> inline
   T* threshold(T* data, int sz, T threshold)
   {
      if(sz == 0) return NULL;

      T* result = allocate<T>(sz);

      for(int i=0; i<sz; i++)
      {
         if( data[i] > threshold ) result[i] = 1;
         else                      result[i] = 0;
      }

      return result;
   }

   /// returns the sign of a point.
   template<class T> inline
   int sign(T num)
   {
      if( num <  0.0 ) return -1;
      if( num == 0.0 ) return  0;
      if( num >  0.0 ) return  1;
   }

   /// returns the sign array of an array.
   template<class T> inline
   int* sign(T* arr, int sz)
   {
      int* out = allocate<int>(sz);
      for( int k=0; k<sz; k++ )
      {
         out[k] = sign( arr[k] );
      }
      return out;
   }

   template<class T> inline
   int compare( const void* a, const void* b )
   {
      return (int)(*(T*)a - *(T*)b);
   }

   /// sorts the data array "data".
   template<class T> inline
   T* quick_sort( T* data, int dsz, bool in_place=true)
   {
      T* out=NULL;
      if( in_place ) out = data;
      else           out = clone(data, dsz);

      std::qsort( out, dsz, sizeof(T), compare<T> );
      return out;
   }

   template<class T> inline
   T median(T* data, int dsz)
   {
      T* tmp = quick_sort(data, dsz, false);
      T med=0;
      if( dsz%2 == 1 ) med =  tmp[ dsz/2 ];
      else             med = (tmp[ dsz/2 ] + tmp[ dsz/2 - 1 ] ) /2;
      deallocate(tmp);
      return med;
   }

   /// computes the median of the array: destroys the contents of the data array.
   template<typename T> inline
   void median( T* data, int sz, T &medval )
   {
      std::qsort(data, sz, sizeof(T), compare<T> );
      if( sz%2 == 1 ) medval = data[sz/2];
      else            medval = (data[sz/2]+data[sz/2-1])/2;
   }

   template<typename T> inline
   void smooth_median( T* data, int h, int w, int msz, T* out )
   {
      int wsz=(2*msz+1)*(2*msz+1);
      const static int max_buffer_size = 441;
      assert( wsz < max_buffer_size );

      T buffer[max_buffer_size];

      for( int y=0; y<h; y++ )
      {
         for( int x=0; x<w; x++ )
         {
            int cnt = 0;
            for( int r=-msz; r<=msz; r++ )
            {
               int yy = y+r;
               if( yy >= h ) yy = h-1;
               if( yy <  0 ) yy = 0;
               for( int c=-msz; c<=msz; c++ )
               {
                  int xx=x+c;
                  if( xx >= w ) xx = w-1;
                  if( xx <  0 ) xx = 0;

                  buffer[cnt++] = data[yy*w+xx];
               }
            }
            median( buffer, wsz, out[y*w+x] );
         }
      }
   }

   /// multiplies two arrays element by element.
   /// the result is in the first array's type
   template<class T1, class T2> inline
   T1* times( T1* arr1, T2* arr2, int w)
   {
      T1* out = allocate<T1>(w);
      for( int i=0; i<w; i++ )
         out[i] = (T1)(arr1[i]*arr2[i]);
      return out;
   }

   /// multiplies two matrices element by element.
   /// the result is in the first matrix's type
   template<class T1, class T2> inline
   T1** times( T1** mat1, T2** mat2, int h, int w)
   {
      T1** out = allocate<T1*>(h);
      for( int i=0; i<h; i++ )
         out[i] = times( mat1[i], mat2[i], w );
      return out;
   }

   /// convert a ** data to a * data in row-first order.
   /// it uses memcpy, therefore, works for built-in types.
   template<class T> inline
   T* arrayize(T** data, int xsz, int ysz)
   {
      T* out = allocate<T>(xsz*ysz);
      for( int i=0; i<ysz; i++ )
         memcpy(out[i*xsz],data[i],sizeof(T)*xsz);
      return out;
   }

   /// inplace shifting: accepts negative shifts
   template<class T>
   T* shift_array(T* arr, int size, int shift)
   {
      // if shift = 0 -> you can return now
      if( shift == 0 ) return arr;

      T* temp_array = allocate<T>(size);

      // if negative -> compansate
      if( shift < 0 ) shift += size;

      // copy the first portion
      memcpy(temp_array, arr+shift, sizeof(T)*(size-shift) );

      // copy the rest
      memcpy(temp_array+size-shift, arr, sizeof(T)*shift );
      memcpy(arr,temp_array,size);

      deallocate(temp_array);
      return arr;
   }

   /// shifts the contents of the array in segmented regions
   /// i.e: shifts the contents by "shift" in a segment
   /// size = n*segment, n = integer
   template<class T>
   T* segmented_shift_array(T* &arr, int size, int segment, int shift)
   {
      int segment_step = size / segment;

      if( shift == 0 ) return arr;

      for( int s=0; s<size; s += segment_step )
      {
         shift_array(arr+s, segment_step, shift);
      }

      return arr;
   }

   /// counts the number of times the value val occurs in data[]
   template<class T> inline
   int count( T* data, int sz, T val)
   {
      int counter = 0;
      for(int i=0; i<sz; i++)
      {
         if( data[i] == val )
            counter++;
      }
      return counter;
   }

   template<class T1, class T2> inline
   void set(T1* data, int sz, T2 val)
   {
      for( int k=0; k<sz; k++ )
         data[k]=(T1)val;
   }

   template<class T1, class T2> inline
   void set(T1** data, int rsz, int csz, T2 val)
   {
      for( int r=0; r<rsz; r++ )
         for( int c=0; c<csz; c++ )
            data[r][c]=(T1)val;
   }

   /// rotates x1 y1 by theta (in radians)
   template<class T1, class T2> inline
   void rotate( T1 y1, T1 x1, T2& y2, T2& x2, float theta, T1 ty, T1 tx )
   {
      float kos = cos( theta );
      float zin = sin( theta );
      x2 = (T2)( x1*kos - y1*zin );
      y2 = (T2)( x1*zin + y1*kos );
      return;
   }

   /// rotates the image with respect to ry, rx.
   template<class T> inline
   T* rotate( T* imge, int h, int w, float theta, float ry=0, float rx=0 )
   {
      float kos = cos(theta);
      float zin = sin(theta);

      int x, y;

      T* rimge = allocate<T>(h*w);
      initialize(rimge, h*w, 0);

      float ty, tx;
      float ny, nx;

      for( y=0; y<h; y++ )
      {
         for( x=0; x<w; x++ )
         {
            tx = x - rx;
            ty = y - ry;

            nx = (  tx * kos - ty * zin + rx );
            ny = (  tx * zin + ty * kos + ry );

            if( is_inside( nx, 0, w-1, ny, 0, h-1 ) )
               rimge[y*w+x] = (T)bilinear_interpolation(imge, w, nx, ny);
         }
      }
      return rimge;
   }

   /// stretches the image to minI=0 -- maxI=255 range
   template<class T> inline
   T* stretch(T* image, int sz, T val, bool in_place=false)
   {
      // find the min intensity in roi
      T min_inten=INT_MAX;
      T max_inten=INT_MIN;

      for( int k=0; k<sz; k++ )
      {
         if( image[k] <= val ) continue;
         if( image[k] < min_inten ) min_inten = image[k];
         if( image[k] > max_inten ) max_inten = image[k];
      }

      float s = 255.0f/(float)(max_inten-min_inten);

      T* output = NULL;
      if( in_place ) output = image;
      else 	     output = zeros<T>(sz);

      for( int k=0; k<sz; k++ )
      {
         if( image[k] > val )
            output[k] = (T)((image[k]-min_inten) * s);
         else
            output[k] = image[k];
      }

      return output;
   }

   /// returns the number of digits of a number.
   inline int digit_number(int num)
   {
      if( num == 0 ) return 1;

      int counter = 0;
      while( num != 0 )
      {
         num /= 10;
         counter++;
      }

      return counter;
   }

   /// returns the value of a sigmoid spanning miny-maxy with 'rate'
   /// and x-symmetry axis sym_axis.
   /// miny-maxy : the minimum and maximum interval for the y axis.
   /// rate      : the rate at which the sigmoid reaches maxy from miny.
   /// sym_axis  : symmetry axis in the x axis. sig(sx-d)+sig(sx+d) = maxy:
   ///             sum of the y values from the symmetry point makes maxy.
   inline float sigmoid(float x, float miny, float maxy, float rate, float sym_axis)
   {
      float xp = exp(rate*(x-sym_axis));
      return (maxy - miny) * xp / ( xp + 1 ) + miny;
   }

   /// returns the "and" of two boolean arrays.
   inline bool* and_array( bool* a, bool* b, int sz)
   {
      bool* c = allocate<bool>(sz);
      for( int i=0; i<sz; i++ )
         c[i] = a[i] & b[i];
      return c;
   }

   /// returns the "or" of two boolean arrays.
   inline bool* or_array( bool* a, bool* b, int sz)
   {
      bool* c = allocate<bool>(sz);
      for( int i=0; i<sz; i++ )
         c[i] = a[i] | b[i];
      return c;
   }

   /// finds the n local-modes: locals -> return indices, workspace[sz]
   template<typename T> inline
   void find_n_local_min(const T* arr, const int sz, int* locals, const int n, T* workspace )
   {
      int min_count=0;

      for( int i=0; i<sz; i++ ) workspace[i] = -1;
      for( int i=0; i<n;  i++ ) locals[i] = -1;

      T prev=INT_MAX;
      T next=INT_MAX;
      for( int i=0; i<sz; i++ )
      {
         if( i > 0    ) prev = arr[i-1]; else prev = INT_MAX;
         if( i < sz-1 ) next = arr[i+1]; else next = INT_MAX;
         if( (arr[i] < prev) && (arr[i] < next) )
         {
            workspace[min_count] = i;
            min_count++;
         }
      }

      // cout<<"mins\n";
      // for( int i=0; i<min_count; i++ )
      // {
      // cout<<workspace[i]<<" ";
      // if( workspace[i] != -1 ) cout<<arr[(int)workspace[i]]<<endl;
      // else                     cout<<-1<<endl;
      // }
      // cout<<endl;

      bool inserted=false;
      int fn=1;
      locals[0] = workspace[0];
      for( int j=1; j<min_count; j++ )
      {
         inserted=false;
         if( workspace[j] == -1 ) break;
         for( int k=0; k<fn; k++ )
         {
            if( arr[ (int)workspace[j] ] <= arr[ locals[k] ] )
            {
               shift_array_right( locals, n, k );
               locals[k] = workspace[j];
               if( fn < n ) fn++;
               inserted=true;
               break;
            }
         }
         if( !inserted && (fn < n) )
         {
            locals[fn] = workspace[j];
            fn++;
         }
      }
      for( int i=fn; i<n; i++ ) locals[i]=-1;

      // cout<<"locals\n";
      // for( int i=0; i<n; i++ )
      // {
      // cout<<locals[i]<<" ";
      // if( locals[i] != -1 ) cout<<arr[locals[i]]<<endl;
      // else                  cout<<-1<<endl;
      // }
      // cout<<endl;
   }
}

#endif
