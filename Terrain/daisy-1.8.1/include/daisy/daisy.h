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

#ifndef DAISY_H
#define DAISY_H

// #include "kutility/kutility.h"

#include "kutility/general.h"
#include "kutility/math.h"
#include "kutility/image.h"
#include "kutility/progress_bar.h"
#include "kutility/fileio.h"
#include "kutility/corecv.h"

using kutility::allocate;
using kutility::deallocate;
using kutility::type_cast;
using kutility::divide;
using kutility::is_outside;
using kutility::save;
using kutility::l2norm;
using kutility::scale;
using kutility::point_transform_via_homography;
using kutility::save_binary;

const double g_sigma_0 = 1;
const double g_sigma_1 = sqrt(2.0);
const double g_sigma_2 = 8;
const double g_sigma_step = std::pow(2,1.0/2);
const int   g_scale_st = int( (log(g_sigma_1/g_sigma_0)) / log(g_sigma_step) );
static int  g_scale_en = 1;

const double g_sigma_init = 1.6;
const static int g_grid_orientation_resolution = 360;

#define MAX_NORMALIZATION_ITER  5
#define MAX_CUBE_NO 64

extern int g_cube_number;
extern int g_selected_cubes[MAX_CUBE_NO]; // m_rad_q_no < MAX_CUBE_NO

enum NRM_TYPE { NRM_PARTIAL, NRM_FULL, NRM_SIFT, NRM_DEFAULT };

class daisy
{
public:
   daisy();
   ~daisy();

   /// if called, I don't use interpolation in the computation of
   /// descriptors.
   void disable_interpolation()
      {
         m_disable_interpolation = true;
      }

   /// returns the size of the descriptor vector
   int descriptor_size() { return m_descriptor_size; }

   /// returns the region number.
   int grid_point_number() { return m_grid_point_number; }

   /// suppresses output messages.
   /// 0: no output
   /// 1: semi-verbose ( important messages only )
   /// 2: full verbosity
   void verbose( size_t verbosity=2 ) { m_verbosity = verbosity; }

   /// releases all the used memory; call this if you want to process
   /// multiple images within a loop.
   void reset();

   /// releases unused memory after descriptor computation is completed.
   void release_auxilary();

   /// computes the descriptors for every pixel in the image.
   void compute_descriptors();

   /// sets image data and size, image is converted to float and normalized.
   template<class T> void set_image(T* im, int h, int w)
      {
         m_h = h;
         m_w = w;
         m_image = type_cast<float,T>(im,h*w);
         divide(m_image, h*w, (float)255.0);
         if( m_verbosity > 3 ) {
            cout<<"[set_image] saving input.bin\n";
            save_binary("input.bin",m_image, m_h, m_w, 1, kutility::TYPE_FLOAT);
         }

      }

   /// sets the descriptor parameters
   void set_parameters( double rad, int rad_q_no, int th_q_no, int hist_th_q_no );

   /// initializes for get_descriptor(double, double, int) mode: pre-computes
   /// convolutions of gradient layers in m_smoothed_gradient_layers
   void initialize_single_descriptor_mode();

   /// returns all the descriptors.
   float* get_dense_descriptors();

   /// returns the used grid point coordinates.
   double** get_grid_points();

   /// returns oriented grid points. default is 0 orientation.
   double* get_grid(int o=0);

   /// saves the descriptor (y,x) to file "filename"
   void save_descriptor( std::string filename, int y, int x, bool single_row=false);

   void save_descriptors( std::string filename )
      {
         cout<<"\n\nDEPRACATED\n\n";
         cout<<"use save_descriptors_ascii or save_descriptors_binary instead\n";
         save_descriptors_ascii(filename);
      }

   /// saves all the descriptors to file "filename" in ascii format
   void save_descriptors_ascii( std::string filename );

   /// saves all the descriptors to file "filename" in binary format
   void save_descriptors_binary( std::string filename );

   /// tells the destructor not to deallocate the memory for the
   /// m_dense_descriptors after the daisy object is destroyed.
   void detach_dense_descriptor_array();

   /// EXPERIMENTAL: DO NOT USE IF YOU ARE NOT ENGIN TOLA: tells to compute the
   /// scales for every pixel so that the resulting descriptors are scale
   /// invariant.
   void scale_invariant( bool state=true )
      {
         g_scale_en = (int)( (log(g_sigma_2/g_sigma_0)) / log(g_sigma_step) ) - g_scale_st;
         m_scale_invariant = state;
      }

   /// EXPERIMENTAL: DO NOT USE IF YOU ARE NOT ENGIN TOLA: tells to compute the
   /// orientations for every pixel so that the resulting descriptors are
   /// rotation invariant. orientation steps are 360/ori_resolution
   void rotation_invariant(int ori_resolution=36, bool state=true)
      {
         m_rotation_invariant = state;
         m_orientation_resolution = ori_resolution;
      }

   /// sets the gaussian variances manually. must be called before
   /// initialize() to be considered. must be exact sigma values -> f
   /// converts to incremental format.
   void set_cube_gaussians( double* sigma_array, int sz );

   int* get_orientation_map() { return m_orientation_map; }

   /// call compute_descriptor_memory to find the amount of memory to allocate
   void set_descriptor_memory( float* descriptor, long int d_size );

   /// call compute_workspace_memory to find the amount of memory to allocate
   void set_workspace_memory( float* workspace, long int w_size );

   /// returns the amount of memory needed for the compute_descriptors()
   /// function. it is basically equal to imagesize x descriptor_size
   int compute_descriptor_memory() {
      if( m_h == 0 || m_descriptor_size == 0 ) {
         cout<<"[daisy] call set_parameters and set_image first!\n";
      }
      return m_w * m_h * m_descriptor_size;
   }

   /// returns the amount of memory needed for workspace. call before initialize()
   int compute_workspace_memory() {
      if( m_cube_size == 0 ) {
         cout<<"[daisy] call set_parameters and set_image first!\n";
      }
      return (g_cube_number+1)* m_cube_size;
   }

   void normalize_descriptor(float* desc, int nrm_type=NRM_DEFAULT)
      {
         if( nrm_type == NRM_DEFAULT )      nrm_type = m_nrm_type;
         if     ( nrm_type == NRM_PARTIAL ) normalize_partial(desc);
         else if( nrm_type == NRM_FULL    ) normalize_full(desc);
         else if( nrm_type == NRM_SIFT    ) normalize_sift_way(desc);
         else
            kutility::error( "[normalized_descriptor] no such normalization" );
      }

   /// returns the descriptor vector for the point (y, x) !!! use this for
   /// precomputed operations meaning that you must call compute_descriptors()
   /// before calling this function. if you want normalized descriptors, call
   /// normalize_descriptors() before calling compute_descriptors()
   inline void get_descriptor(int y, int x, float* &descriptor);

   /// computes the descriptor and returns the result in 'descriptor' ( allocate
   /// 'descriptor' memory first ie: float descriptor = new
   /// float[m_descriptor_size]; -> the descriptor is normalized.
   inline void get_descriptor(double y, double x, int orientation, float* descriptor );

   /// computes the descriptor and returns the result in 'descriptor' ( allocate
   /// 'descriptor' memory first ie: float descriptor = new
   /// float[m_descriptor_size]; -> the descriptor is NOT normalized.
   inline void get_unnormalized_descriptor(double y, double x, int orientation, float* descriptor );

   /// computes the descriptor at homography-warped grid. (y,x) is not the
   /// coordinates of this image but the coordinates of the original grid where
   /// the homography will be applied. Meaning that the grid is somewhere else
   /// and we warp this grid with H and compute the descriptor on this warped
   /// grid; returns null/false if centers falls outside the image; allocate
   /// 'descriptor' memory first. descriptor is normalized.
   inline bool get_descriptor(double y, double x, int orientation, double* H, float* descriptor );

   /// computes the descriptor at homography-warped grid. (y,x) is not the
   /// coordinates of this image but the coordinates of the original grid where
   /// the homography will be applied. Meaning that the grid is somewhere else
   /// and we warp this grid with H and compute the descriptor on this warped
   /// grid; returns null/false if centers falls outside the image; allocate
   /// 'descriptor' memory first. descriptor is NOT normalized.
   inline bool get_unnormalized_descriptor(double y, double x, int orientation, double* H, float* descriptor );

   int get_hq() { return m_hist_th_q_no; }
   int get_thq() { return m_th_q_no; }
   int get_rq() { return m_rad_q_no; }
   float get_rad() { return m_rad; }

   /// sets the type of the normalization to apply out of {NRM_PARTIAL,
   /// NRM_FULL, NRM_SIFT}. Call before using get_descriptor() if you want to
   /// change the default normalization type.
   void set_normalization( int nrm_type ) {
         assert( nrm_type != NRM_DEFAULT );
         m_nrm_type = nrm_type;
      }

   /// applies one of the normalizations (partial,full,sift) to the desciptors.
   void normalize_descriptors(int nrm_type=NRM_DEFAULT);

   /// normalizes histograms individually
   void normalize_histograms();

   /// gets the histogram at y,x with 'orientation' from the r'th cube
   inline float* get_histogram( int y, int x, int r );

private:

   /// maximum radius of the descriptor region.
   float m_rad;

   /// the number of quantizations of the radius.
   int m_rad_q_no;

   /// the number of quantizations of the angle.
   int m_th_q_no;

   /// the number of quantizations of the gradient orientations.
   int m_hist_th_q_no;

   /// holds the type of the normalization to apply; equals to NRM_PARTIAL by
   /// default. change the value using set_normalization() function.
   int m_nrm_type;

   /// computes the histogram at yx; the size of histogram is m_hist_th_q_no
   void compute_histogram( float* hcube, int y, int x, float* histogram );

   /// reorganizes the cube data so that histograms are sequential in memory.
   void compute_histograms();

   /// uses interpolation, for no interpolation call ni_get_descriptor. see also get_descriptor
   inline void i_get_descriptor(double y, double x, int orientation, float* descriptor );

   /// does not use interpolation. for w/interpolation, call i_get_descriptor. see also get_descriptor
   inline void ni_get_descriptor(double y, double x, int orientation, float* descriptor );

   /// uses interpolation for no interpolation call ni_get_descriptor. see also get_descriptor
   inline bool i_get_descriptor(double y, double x, int orientation, double* H, float* descriptor );

   /// does not use interpolation. for w/interpolation, call i_get_descriptor. see also get_descriptor
   inline bool ni_get_descriptor(double y, double x, int orientation, double* H, float* descriptor );

   /// emulates the way sift is normalized.
   void normalize_sift_way( float* desc );

   /// normalizes the descriptor histogram by histogram
   void normalize_partial ( float* desc );

   /// normalizes the full descriptor.
   void normalize_full    ( float* desc );

   /// initializes the class: computes gradient and structure-points
   void initialize();

   void update_selected_cubes();

   int quantize_radius( float rad );

   /// compute the smoothed gradient layers.
   inline void compute_smoothed_gradient_layers();

   /// does not use interpolation while computing the histogram.
   inline void ni_get_histogram( float* histogram, int y, int x, int shift, float* hcube );

   /// returns the interpolated histogram: picks either bi_get_histogram or
   /// ti_get_histogram depending on 'shift'
   inline void i_get_histogram( float* histogram, double y, double x, double shift, float* cube );

   /// records the histogram that is computed by bilinear interpolation
   /// regarding the shift in the spatial coordinates. hcube is the
   /// histogram cube for a constant smoothness level.
   inline void bi_get_histogram( float* descriptor, double y, double x, int shift, float* hcube );

   /// records the histogram that is computed by trilinear interpolation
   /// regarding the shift in layers and spatial coordinates. hcube is the
   /// histogram cube for a constant smoothness level.
   inline void ti_get_histogram( float* descriptor, double y, double x, double shift, float* hcube );

   int filter_size( double sigma );

   /// computes scales for every pixel and scales the structure grid so that the
   /// resulting descriptors are scale invariant.  you must set
   /// m_scale_invariant flag to 1 for the program to call this function
   void compute_scales();

   /// Return a number in the range [-0.5, 0.5] that represents the location of
   /// the peak of a parabola passing through the 3 evenly spaced samples.  The
   /// center value is assumed to be greater than or equal to the other values
   /// if positive, or less than if negative.
   float interpolate_peak(float left, float center, float right);

   /// Smooth a histogram by using a [1/3 1/3 1/3] kernel.  Assume the histogram
   /// is connected in a circular buffer.
   void smooth_histogram(float *hist, int bins);

   /// computes pixel orientations and rotates the structure grid so that
   /// resulting descriptors are rotation invariant. If the scales is also
   /// detected, then orientations are computed at the computed scales. you must
   /// set m_rotation_invariant flag to 1 for the program to call this function
   void compute_orientations();

   /// the clipping threshold to use in normalization: values above this value
   /// are clipped to this value for normalize_sift_way() function
   float m_descriptor_normalization_threshold;

   /// computes the sigma's of layers from descriptor parameters if the user did
   /// not sets it. these define the size of the petals of the descriptor.
   void compute_cube_sigmas();

   /// Computes the locations of the unscaled unrotated points where the
   /// histograms are going to be computed according to the given parameters.
   void compute_grid_points();

   /// Computes the locations of the unscaled rotated points where the
   /// histograms are going to be computed according to the given parameters.
   void compute_oriented_grid_points();

   /// Sets the locations of the unscaled unrotated points where the histograms
   /// are going to be computed. Call this function before initializion.
   void set_grid_points();

   /// smooths each of the layers by a Gaussian having "sigma" standart
   /// deviation.
   void smooth_layers( float*layers, int h, int w, int layer_number, float sigma );

   /// Holds the coordinates (y,x) of the grid points of the region.
   double** m_grid_points;

   /// if set to true, no verbose information is printed. should change for
   /// different levels of verbosity.
   size_t m_verbosity;

   /// input image.
   float* m_image;

   /// image height
   int m_h;

   /// image width
   int m_w;

   /// if set to false, destructor won't delete m_dense_descriptors.
   bool m_release_descriptors;

   /// stores the descriptors : its size is [ m_w * m_h * m_descriptor_size ].
   float* m_dense_descriptors;

   /// stores the layered gradients in successively smoothed form: layer[n] =
   /// m_gradient_layers * gaussian( sigma_n ); n>= 1; layer[0] is the layered_gradient
   float* m_smoothed_gradient_layers;

   /// if set to true, descriptors are scale invariant
   bool m_scale_invariant;

   /// if set to true, descriptors are rotation invariant
   bool m_rotation_invariant;

   /// number of bins in the histograms while computing orientation
   int m_orientation_resolution;

   /// hold the scales of the pixels
   float* m_scale_map;

   /// holds the orientaitons of the pixels
   int* m_orientation_map;

   /// Holds the oriented coordinates (y,x) of the grid points of the region.
   double** m_oriented_grid_points;

   /// holds the gaussian sigmas for radius quantizations for an incremental
   /// application
   double* m_cube_sigmas;

   bool m_descriptor_memory;
   bool m_workspace_memory;

   /// the number of grid locations
   int m_grid_point_number;

   /// the size of the descriptor vector
   int m_descriptor_size;

   /// holds the amount of shift that's required for histogram computation
   double m_orientation_shift_table[360];

   /// if enabled, descriptors are computed with casting non-integer locations
   /// to integer positions otherwise we use interpolation.
   bool m_disable_interpolation;

   /// size of m_hsz layers at a single sigma: m_hsz * m_layer_size
   int m_cube_size;

   /// size of the layer : m_h*m_w
   int m_layer_size;
};

inline void daisy::compute_histogram( float* hcube, int y, int x, float* histogram )
{
   if( is_outside(x, 0, m_w-1, y, 0, m_h-1) ) return;

   float* spatial_shift = hcube + y * m_w + x;
   int data_size =  m_w * m_h;

   for( int h=0; h<m_hist_th_q_no; h++ )
      histogram[h] = *(spatial_shift + h*data_size);
}

float* daisy::get_histogram( int y, int x, int r )
{
   assert( y >= 0 && y < m_h );
   assert( x >= 0 && x < m_w );
   assert( m_smoothed_gradient_layers );
   assert( m_oriented_grid_points );
   return m_smoothed_gradient_layers+g_selected_cubes[r]*m_cube_size + (y*m_w+x)*m_hist_th_q_no;
   // i_get_histogram( histogram, y, x, 0, m_smoothed_gradient_layers+g_selected_cubes[r]*m_cube_size );
}

inline void daisy:: i_get_histogram( float* histogram, double y, double x, double shift, float* cube )
{
   int ishift=(int)shift;
   double fshift=shift-ishift;
   if     ( fshift < 0.01 ) bi_get_histogram( histogram, y, x, ishift  , cube );
   else if( fshift > 0.99 ) bi_get_histogram( histogram, y, x, ishift+1, cube );
   else                     ti_get_histogram( histogram, y, x,  shift  , cube );
}
inline void daisy::bi_get_histogram( float* histogram, double y, double x, int shift, float* hcube )
{
   int mnx = int( x );
   int mny = int( y );

   if( mnx >= m_w-2  || mny >= m_h-2 )
   {
      memset(histogram, 0, sizeof(float)*m_hist_th_q_no);
      return;
   }

   int ind =  mny*m_w+mnx;
   // A C --> pixel positions
   // B D
   float* A = hcube+ind*m_hist_th_q_no;
   float* B = A+m_w*m_hist_th_q_no;
   float* C = A+m_hist_th_q_no;
   float* D = A+(m_w+1)*m_hist_th_q_no;

   double alpha = mnx+1-x;
   double beta  = mny+1-y;

   float w0 = alpha*beta;
   float w1 = beta-w0; // (1-alpha)*beta;
   float w2 = alpha-w0; // (1-beta)*alpha;
   float w3 = 1+w0-alpha-beta; // (1-beta)*(1-alpha);

   int h;

   for( h=0; h<m_hist_th_q_no; h++ ) {
      if( h+shift < m_hist_th_q_no ) histogram[h] = w0*A[h+shift];
      else                           histogram[h] = w0*A[h+shift-m_hist_th_q_no];
   }
   for( h=0; h<m_hist_th_q_no; h++ ) {
      if( h+shift < m_hist_th_q_no ) histogram[h] += w1*C[h+shift];
      else                           histogram[h] += w1*C[h+shift-m_hist_th_q_no];
   }
   for( h=0; h<m_hist_th_q_no; h++ ) {
      if( h+shift < m_hist_th_q_no ) histogram[h] += w2*B[h+shift];
      else                           histogram[h] += w2*B[h+shift-m_hist_th_q_no];
   }
   for( h=0; h<m_hist_th_q_no; h++ ) {
      if( h+shift < m_hist_th_q_no ) histogram[h] += w3*D[h+shift];
      else                           histogram[h] += w3*D[h+shift-m_hist_th_q_no];
   }
}
inline void daisy::ti_get_histogram( float* histogram, double y, double x, double shift, float* hcube )
{
   int ishift = int( shift );
   double layer_alpha  = shift - ishift;

   float thist[MAX_CUBE_NO];
   bi_get_histogram( thist, y, x, ishift, hcube );

   for( int h=0; h<m_hist_th_q_no-1; h++ )
      histogram[h] = (1-layer_alpha)*thist[h]+layer_alpha*thist[h+1];
   histogram[m_hist_th_q_no-1] = (1-layer_alpha)*thist[m_hist_th_q_no-1]+layer_alpha*thist[0];
}
inline void daisy::ni_get_histogram( float* histogram, int y, int x, int shift, float* hcube )
{
   if( is_outside(x, 0, m_w-1, y, 0, m_h-1) ) return;
   float* hptr = hcube + (y*m_w+x)*m_hist_th_q_no;

   for( int h=0; h<m_hist_th_q_no; h++ )
   {
      int hi = h+shift;
      if( hi >= m_hist_th_q_no ) hi -= m_hist_th_q_no;
      histogram[h] = hptr[hi];
   }
}

inline void daisy::get_descriptor(int y, int x, float* &descriptor)
{
   assert( m_dense_descriptors != NULL );
   assert( y<m_h && x<m_w && y>=0 && x>=0 );
   descriptor = &(m_dense_descriptors[(y*m_w+x)*m_descriptor_size]);
}

inline void daisy::get_descriptor(double y, double x, int orientation, float* descriptor )
{
   get_unnormalized_descriptor(y, x, orientation, descriptor );
   normalize_descriptor(descriptor, m_nrm_type);
}
inline void daisy::get_unnormalized_descriptor(double y, double x, int orientation, float* descriptor )
{
   if( m_disable_interpolation ) ni_get_descriptor(y,x,orientation,descriptor);
   else                           i_get_descriptor(y,x,orientation,descriptor);
}

inline void daisy:: i_get_descriptor(double y, double x, int orientation, float* descriptor )
{
   // memset( descriptor, 0, sizeof(float)*m_descriptor_size );
   //
   // i'm not changing the descriptor[] values if the gridpoint is outside
   // the image. you should memset the descriptor array to 0 if you don't
   // want to have stupid values there.
   //
   assert( y >= 0 && y < m_h );
   assert( x >= 0 && x < m_w );
   assert( orientation >= 0 && orientation < 360 );
   assert( m_smoothed_gradient_layers );
   assert( m_oriented_grid_points );
   assert( descriptor != NULL );

   double shift = m_orientation_shift_table[orientation];

   i_get_histogram( descriptor, y, x, shift, m_smoothed_gradient_layers+g_selected_cubes[0]*m_cube_size );

   int r, rdt, region;
   double yy, xx;
   float* histogram = 0;
   double* grid = m_oriented_grid_points[orientation];

   // petals of the flower
   for( r=0; r<m_rad_q_no; r++ )
   {
      rdt  = r*m_th_q_no+1;
      for( region=rdt; region<rdt+m_th_q_no; region++ )
      {
         yy = y + grid[2*region  ];
         xx = x + grid[2*region+1];
         if( is_outside(xx, 0, m_w-1, yy, 0, m_h-1) ) continue;
         histogram = descriptor+region*m_hist_th_q_no;
         i_get_histogram( histogram, yy, xx, shift, m_smoothed_gradient_layers+g_selected_cubes[r]*m_cube_size );
      }
   }
}
inline void daisy::ni_get_descriptor(double y, double x, int orientation, float* descriptor )
{
   // memset( descriptor, 0, sizeof(float)*m_descriptor_size );
   //
   // i'm not changing the descriptor[] values if the gridpoint is outside
   // the image. you should memset the descriptor array to 0 if you don't
   // want to have stupid values there.
   //
   assert( y >= 0 && y < m_h );
   assert( x >= 0 && x < m_w );
   assert( orientation >= 0 && orientation < 360 );
   assert( m_smoothed_gradient_layers );
   assert( m_oriented_grid_points );
   assert( descriptor != NULL );

   double shift = m_orientation_shift_table[orientation];
   int ishift = (int)shift;
   if( shift - ishift > 0.5  ) ishift++;

   int iy = (int)y; if( y - iy > 0.5 ) iy++;
   int ix = (int)x; if( x - ix > 0.5 ) ix++;

   // center
   ni_get_histogram( descriptor, iy, ix, ishift, m_smoothed_gradient_layers+g_selected_cubes[0]*m_cube_size );

   double yy, xx;
   float* histogram=0;
   // petals of the flower
   int r, rdt, region;
   double* grid = m_oriented_grid_points[orientation];
   for( r=0; r<m_rad_q_no; r++ )
   {
      rdt = r*m_th_q_no+1;
      for( region=rdt; region<rdt+m_th_q_no; region++ )
      {
         yy = y + grid[2*region  ];
         xx = x + grid[2*region+1];
         iy = (int)yy; if( yy - iy > 0.5 ) iy++;
         ix = (int)xx; if( xx - ix > 0.5 ) ix++;

         if( is_outside(ix, 0, m_w-1, iy, 0, m_h-1) ) continue;

         histogram = descriptor+region*m_hist_th_q_no;
         ni_get_histogram( histogram, iy, ix, ishift, m_smoothed_gradient_layers+g_selected_cubes[r]*m_cube_size );
      }
   }
}

// Warped get_descriptor's
inline bool daisy::get_descriptor(double y, double x, int orientation, double* H, float* descriptor )
{
   bool rval = get_unnormalized_descriptor(y,x,orientation, H, descriptor);
   if( rval ) normalize_descriptor(descriptor, m_nrm_type);
   return rval;
}
inline bool daisy::get_unnormalized_descriptor(double y, double x, int orientation, double* H, float* descriptor )
{
   if( m_disable_interpolation ) return ni_get_descriptor(y,x,orientation,H,descriptor);
   else                          return   i_get_descriptor(y,x,orientation,H,descriptor);
}
inline bool daisy:: i_get_descriptor(double y, double x, int orientation, double* H, float* descriptor )
{
   // memset( descriptor, 0, sizeof(float)*m_descriptor_size );
   //
   // i'm not changing the descriptor[] values if the gridpoint is outside
   // the image. you should memset the descriptor array to 0 if you don't
   // want to have stupid values there.
   //
   assert( orientation >= 0 && orientation < 360 );
   assert( m_smoothed_gradient_layers );
   assert( descriptor != NULL );

   int hradius[MAX_CUBE_NO];

   double hy, hx, ry, rx;
   point_transform_via_homography(H, x, y, hx, hy );
   if( is_outside( hx, 0, m_w, hy, 0, m_h ) ) return false;

   point_transform_via_homography(H, x+m_cube_sigmas[g_selected_cubes[0]], y, rx, ry);
   double radius =  l2norm( ry, rx, hy, hx );
   hradius[0] = quantize_radius( radius );

   double shift = m_orientation_shift_table[orientation];
   i_get_histogram( descriptor, hy, hx, shift, m_smoothed_gradient_layers+hradius[0]*m_cube_size );

   double gy, gx;
   int r, rdt, th, region;
   float* histogram=0;
   for( r=0; r<m_rad_q_no; r++)
   {
      rdt = r*m_th_q_no + 1;
      for( th=0; th<m_th_q_no; th++ )
      {
         region = rdt + th;

         gy = y + m_grid_points[region][0];
         gx = x + m_grid_points[region][1];

         point_transform_via_homography(H, gx, gy, hx, hy);
         if( th == 0 )
         {
            point_transform_via_homography(H, gx+m_cube_sigmas[g_selected_cubes[r]], gy, rx, ry);
            radius = l2norm( ry, rx, hy, hx );
            hradius[r] = quantize_radius( radius );
         }

         if( is_outside(hx, 0, m_w-1, hy, 0, m_h-1) ) continue;

         histogram = descriptor+region*m_hist_th_q_no;
         i_get_histogram( histogram, hy, hx, shift, m_smoothed_gradient_layers+hradius[r]*m_cube_size );
      }
   }
   return true;
}
inline bool daisy::ni_get_descriptor(double y, double x, int orientation, double* H, float* descriptor )
{
   // memset( descriptor, 0, sizeof(float)*m_descriptor_size );
   //
   // i'm not changing the descriptor[] values if the gridpoint is outside
   // the image. you should memset the descriptor array to 0 if you don't
   // want to have stupid values there.
   //
   assert( orientation >= 0 && orientation < 360 );
   assert( m_smoothed_gradient_layers );
   assert( descriptor != NULL );

   int hradius[MAX_CUBE_NO];
   double radius;

   double hy, hx, ry, rx;

   point_transform_via_homography(H, x, y, hx, hy );
   if( is_outside( hx, 0, m_w, hy, 0, m_h ) ) return false;

   double shift = m_orientation_shift_table[orientation];
   int  ishift = (int)shift; if( shift - ishift > 0.5  ) ishift++;

   point_transform_via_homography(H, x+m_cube_sigmas[g_selected_cubes[0]], y, rx, ry);
   radius =  l2norm( ry, rx, hy, hx );
   hradius[0] = quantize_radius( radius );

   int ihx = (int)hx; if( hx - ihx > 0.5 ) ihx++;
   int ihy = (int)hy; if( hy - ihy > 0.5 ) ihy++;

   int r, rdt, th, region;
   double gy, gx;
   float* histogram=0;
   ni_get_histogram( descriptor, ihy, ihx, ishift, m_smoothed_gradient_layers+hradius[0]*m_cube_size );
   for( r=0; r<m_rad_q_no; r++)
   {
      rdt = r*m_th_q_no + 1;
      for( th=0; th<m_th_q_no; th++ )
      {
         region = rdt + th;

         gy = y + m_grid_points[region][0];
         gx = x + m_grid_points[region][1];

         point_transform_via_homography(H, gx, gy, hx, hy);
         if( th == 0 )
         {
            point_transform_via_homography(H, gx+m_cube_sigmas[g_selected_cubes[r]], gy, rx, ry);
            radius = l2norm( ry, rx, hy, hx );
            hradius[r] = quantize_radius( radius );
         }

         ihx = (int)hx; if( hx - ihx > 0.5 ) ihx++;
         ihy = (int)hy; if( hy - ihy > 0.5 ) ihy++;

         if( is_outside(ihx, 0, m_w-1, ihy, 0, m_h-1) ) continue;
         histogram = descriptor+region*m_hist_th_q_no;
         ni_get_histogram( histogram, ihy, ihx, ishift, m_smoothed_gradient_layers+hradius[r]*m_cube_size );
      }
   }
   return true;
}


#endif
