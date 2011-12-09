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

#include <iostream>
#include "daisy/daisy.h"

using namespace std;
using namespace kutility;

int g_cube_number = 3;
int g_selected_cubes[MAX_CUBE_NO];


daisy::daisy()
{
   m_verbosity = 2;
   m_image = 0;
   m_w = 0;
   m_h = 0;

   m_rad = 0;
   m_rad_q_no = 0;
   m_th_q_no  = 0;
   m_hist_th_q_no = 0;
   m_grid_point_number = 0;
   m_descriptor_size = 0;

   m_smoothed_gradient_layers = NULL;
   m_dense_descriptors   = NULL;
   m_grid_points = NULL;
   m_oriented_grid_points = NULL;

   m_scale_invariant = false;
   m_rotation_invariant = false;

   m_scale_map = NULL;
   m_orientation_map = NULL;
   m_orientation_resolution = 36;
   m_scale_map = NULL;

   m_cube_sigmas = NULL;

   m_descriptor_memory = false;
   m_workspace_memory = false;
   m_descriptor_normalization_threshold = 0.154; // sift magical number

   m_disable_interpolation = false;

   m_nrm_type = NRM_PARTIAL;

   m_cube_size = 0;
   m_layer_size = 0;
}

daisy::~daisy()
{
   deallocate( m_image );

   if( !m_workspace_memory ) deallocate( m_smoothed_gradient_layers );
   deallocate( m_grid_points, m_grid_point_number );
   deallocate( m_oriented_grid_points, g_grid_orientation_resolution );
   deallocate( m_orientation_map );
   deallocate( m_scale_map );
   deallocate( m_cube_sigmas );
   if( !m_descriptor_memory ) deallocate( m_dense_descriptors );
}

void daisy::set_parameters( double rad, int rad_q_no, int th_q_no, int hist_th_q_no )
{
   m_rad = rad;                   // radius of the descriptor at the initial scale
   m_rad_q_no = rad_q_no;         // how many pieces shall I divide the radial range ?
   m_th_q_no = th_q_no;           // how many pieces shall I divide the angular range  ?
   m_hist_th_q_no = hist_th_q_no; // how many pieces shall I divide the grad_hist
   m_grid_point_number = m_rad_q_no * m_th_q_no + 1; // +1 is for center pixel
   m_descriptor_size = m_grid_point_number * m_hist_th_q_no;

   for( int i=0; i<360; i++ )
   {
      m_orientation_shift_table[i] = i/360.0 * m_hist_th_q_no;
   }
   m_layer_size = m_h*m_w;
   m_cube_size = m_layer_size*m_hist_th_q_no;

   compute_cube_sigmas();
   compute_grid_points();
}

float* daisy::get_dense_descriptors()
{
   return m_dense_descriptors;
}

double** daisy::get_grid_points()
{
   cout<<"[depracated] use get_grid() instead\n";
   return m_grid_points;
}

double* daisy::get_grid(int o)
{
   assert( o >= 0 && o < 360 );
   return m_oriented_grid_points[o];
}

void daisy::reset()
{
   deallocate( m_image );
   // deallocate( m_grid_points, m_grid_point_number );
   // deallocate( m_oriented_grid_points, g_grid_orientation_resolution );
   // deallocate( m_cube_sigmas );
   deallocate( m_orientation_map );
   deallocate( m_scale_map );
   if( !m_descriptor_memory ) deallocate( m_dense_descriptors );
   if( !m_workspace_memory ) deallocate(m_smoothed_gradient_layers);
}

void daisy::release_auxilary()
{
   deallocate( m_image );
   deallocate( m_orientation_map );
   deallocate( m_scale_map );

   if( !m_workspace_memory ) deallocate(m_smoothed_gradient_layers);

   deallocate( m_grid_points, m_grid_point_number );
   deallocate( m_oriented_grid_points, g_grid_orientation_resolution );
   deallocate( m_cube_sigmas );
}

void daisy::compute_grid_points()
{
   double r_step = m_rad / m_rad_q_no;
   double t_step = 2*PI/ m_th_q_no;

   if( m_grid_points )
      deallocate( m_grid_points, m_grid_point_number );

   m_grid_points = allocate<double>(m_grid_point_number, 2);
   for( int y=0; y<m_grid_point_number; y++ )
   {
      m_grid_points[y][0] = 0;
      m_grid_points[y][1] = 0;
   }

   for( int r=0; r<m_rad_q_no; r++ )
   {
      int region = r*m_th_q_no+1;
      for( int t=0; t<m_th_q_no; t++ )
      {
         double y, x;
         polar2cartesian( (r+1)*r_step, t*t_step, y, x );
         m_grid_points[region+t][0] = y;
         m_grid_points[region+t][1] = x;
      }
   }

   if( m_verbosity > 2 )
   {
      cout<<"[daisy] grid points:";
      display( m_grid_points, m_grid_point_number, 2 );
   }
   compute_oriented_grid_points();
}

/// Computes the descriptor by sampling convoluted orientation maps.
void daisy::compute_descriptors()
{
   if( m_verbosity >= 4 ) {
      cout<<"#######################################################################################\n";
      cout<<"as of version 1.5, compute_descriptors() does not return normalized descriptors.\n";
      cout<<"call normalize_descriptors() after compute_descriptors() if you want normalization.\n";
      cout<<"additionally, you may change the normalization algorithm by calling set_normalization()\n";
      cout<<"#######################################################################################\n";
   }
   if( m_scale_invariant    ) compute_scales();
   if( m_rotation_invariant ) compute_orientations();
   if( !m_descriptor_memory ) m_dense_descriptors = allocate <float>(m_h*m_w*m_descriptor_size);

   memset(m_dense_descriptors, 0, sizeof(float)*m_h*m_w*m_descriptor_size);

   int progress_counter=0;
   progress_bar bar( 0, m_h, 20);
   bar.set_text("[daisy] ");

   int y, x, index, orientation;
#ifdef USE_OPENMP
#pragma omp parallel for private(y,x,index,orientation)
#endif
   for( y=0; y<m_h; y++ )
   {
      if( m_verbosity > 0 ) cout<<bar(++progress_counter)<<flush;
      for( x=0; x<m_w; x++ )
      {
         index=y*m_w+x;
         orientation=0;
         if( m_orientation_map ) orientation = m_orientation_map[index];
         if( !( orientation >= 0 && orientation < g_grid_orientation_resolution ) ) orientation = 0;
         get_unnormalized_descriptor( y, x, orientation, &(m_dense_descriptors[index*m_descriptor_size]) );
      }
   }
}

void daisy::smooth_layers( float* layers, int h, int w, int layer_number, float sigma )
{
   int fsz = filter_size(sigma);
   float* filter = new float[fsz];
   // cout<<"smooth sigma: "<<sigma<<endl;
   gaussian_1d(filter, fsz, sigma, 0);
   int i;
   float* layer=0;
#ifdef USE_OPENMP
#pragma omp parallel for private(i, layer)
#endif
   for( i=0; i<layer_number; i++ )
   {
      layer = layers + i*h*w;
      convolve_sym( layer, h, w, filter, fsz );
   }
   deallocate(filter);
}

void daisy::save_descriptor( string filename, int y, int x, bool single_row)
{
   float* feat = &(m_dense_descriptors[(y*m_w+x)*m_descriptor_size]);
   if( single_row ) save( filename, feat, 1, m_descriptor_size );
   else             save( filename, feat, m_grid_point_number , m_hist_th_q_no );
}

void daisy::save_descriptors_ascii( string filename )
{
   kutility::save(filename, m_dense_descriptors, m_h*m_w, m_descriptor_size );
}

void daisy::save_descriptors_binary( string filename )
{
   kutility::save_binary(filename, m_dense_descriptors, m_h*m_w, m_descriptor_size, 1, kutility::TYPE_FLOAT );
}

void daisy::normalize_partial( float* desc )
{
   float norm;
   for( int h=0; h<m_grid_point_number; h++ )
   {
      norm =  l2norm( &(desc[h*m_hist_th_q_no]), m_hist_th_q_no );
      if( norm != 0.0 ) divide( desc+h*m_hist_th_q_no, m_hist_th_q_no, norm);
   }
}
void daisy::normalize_full( float* desc )
{
   float norm =  l2norm( desc, m_descriptor_size );
   if( norm != 0.0 ) divide(desc, m_descriptor_size, norm);
}
void daisy::normalize_sift_way( float* desc )
{
   bool changed = true;
   int iter = 0;
   float norm;
   int h;
   while( changed && iter < MAX_NORMALIZATION_ITER )
   {
      iter++;
      changed = false;

      norm = l2norm( desc, m_descriptor_size );
      if( norm > 1e-5 )
         divide( desc, m_descriptor_size, norm);

      for( h=0; h<m_descriptor_size; h++ )
      {
         if( desc[ h ] > m_descriptor_normalization_threshold )
         {
            desc[ h ] = m_descriptor_normalization_threshold;
            changed = true;
         }
      }
   }
}
void daisy::normalize_descriptors( int nrm_type )
{
   time_t st, en;
   if( m_verbosity > 0 ) {
      cout<<"[daisy] normalizing descriptors...";
      time(&st);
   }
   int number_of_descriptors =  m_h * m_w;
   int d;

#ifdef USE_OPENMP
#pragma omp parallel for private(d)
#endif
   for( d=0; d<number_of_descriptors; d++ )
      normalize_descriptor( m_dense_descriptors+d*m_descriptor_size, nrm_type );
   if( m_verbosity > 0 ) {
      time(&en);
      cout<<" in "<<difftime(en,st)<<" seconds\n";
   }
}

void daisy::initialize_single_descriptor_mode()
{
   initialize();
   compute_smoothed_gradient_layers();
}

void daisy::initialize()
{
   assert(m_h != 0); // call set_image first.
   assert(m_w != 0);

   if( m_layer_size==0 ) {
      m_layer_size = m_h*m_w;
      m_cube_size = m_layer_size*m_hist_th_q_no;
   }

   if( m_verbosity > 0 ) cout<<"\n[daisy] initializing...";

   int glsz = compute_workspace_memory();
   if( !m_workspace_memory ) m_smoothed_gradient_layers = new float[glsz];

   float* gradient_layers = m_smoothed_gradient_layers;

   if( m_verbosity > 3 ) {
      cout<<"\n[initialize] saving input.bin\n";
      save_binary("input.bin",m_image, m_h, m_w, 1, TYPE_FLOAT);
   }

   layered_gradient( m_image, m_h, m_w, m_hist_th_q_no, gradient_layers );
   if( m_verbosity == 4 ) {
      for( int ii=0; ii<m_hist_th_q_no; ii++ ) {
         string file = "gradient_layers"+num2str(ii)+".bin";
         cout<<"[initialize] saving "<<file<<endl;
         save_binary(file,gradient_layers+ii*m_h*m_w, m_h, m_w, 1, TYPE_FLOAT);
      }
   }

   // assuming a 0.5 image smoothness, we pull this to 1.6 as in sift
   smooth_layers( gradient_layers, m_h, m_w, m_hist_th_q_no, sqrt(g_sigma_init*g_sigma_init-0.25) );

   if( m_verbosity == 4 ) {
      for( int ii=0; ii<m_hist_th_q_no; ii++ ) {
         string file = "sgradient_layers"+num2str(ii)+".bin";
         cout<<"[initialize] saving "<<file<<endl;
         save_binary(file,gradient_layers+ii*m_h*m_w, m_h, m_w, 1, TYPE_FLOAT);
      }
   }

   if( m_verbosity > 0 ) cout<<" ok!\n";
}

void daisy::compute_cube_sigmas()
{
   if( m_verbosity > 0 ) cout<<"[daisy] compute_cube_sigmas\n";
   if( m_cube_sigmas == NULL )
   {
      // user didn't set the sigma's; set them from the descriptor parameters
      g_cube_number = m_rad_q_no;
      m_cube_sigmas = allocate<double>(g_cube_number);

      double r_step = double(m_rad)/m_rad_q_no;
      for( int r=0; r< m_rad_q_no; r++ )
      {
         m_cube_sigmas[r] = (r+1)*r_step/2;
         if( m_verbosity > 1 )
            cout<<"[daisy] cube sigma "<<r<<": "<<m_cube_sigmas[r]<<endl;
      }
   }
   update_selected_cubes();
}

void daisy::set_cube_gaussians( double* sigma_array, int sz )
{
   g_cube_number = sz;

   if( m_cube_sigmas ) deallocate( m_cube_sigmas );
   m_cube_sigmas = allocate<double>(g_cube_number);

   for( int r=0; r<g_cube_number; r++ )
   {
      m_cube_sigmas[r] = sigma_array[r];
      if( m_verbosity > 1 ) cout<<"[daisy] sigma"<<r<<": "<<sigma_array[r]<<endl;
   }
   update_selected_cubes();
}

void daisy::update_selected_cubes()
{
   if( m_verbosity > 0 ) cout<<"[daisy] update_selected_cubes\n";

   for( int r=0; r<m_rad_q_no; r++ )
   {
      double seed_sigma = (r+1)*m_rad/m_rad_q_no/2.0;
      g_selected_cubes[r] = quantize_radius( seed_sigma );
      if( m_verbosity > 1 )
      {
         // cout<<"[daisy] seed : "<<seed_sigma<<endl;
         cout<<"[daisy] g_selected_cubes["<<r<<"] = "<<g_selected_cubes[r]<<" sigma: "<<m_cube_sigmas[ g_selected_cubes[r] ] <<endl;
      }
   }
}

int daisy::quantize_radius( float rad )
{
   if( rad <= m_cube_sigmas[0              ] ) return 0;
   if( rad >= m_cube_sigmas[g_cube_number-1] ) return g_cube_number-1;

   float dist;
   float mindist=FLT_MAX;
   int mini=0;
   for( int c=0; c<g_cube_number; c++ ) {
      dist = fabs( m_cube_sigmas[c]-rad );
      if( dist < mindist ) {
         mindist = dist;
         mini=c;
      }
   }
   return mini;
}

void daisy::compute_histograms()
{
   time_t st, en;
   if( m_verbosity > 0 ) {
      cout<<"[daisy] compute_histograms ";
      time(&st);
   }

   int r, y, x, ind;
   float* hist=0;

   for( r=0; r<g_cube_number; r++ )
   {
      float* dst = m_smoothed_gradient_layers+r*m_cube_size;
      float* src = m_smoothed_gradient_layers+(r+1)*m_cube_size;

#ifdef USE_OPENMP
#pragma omp parallel for private(y,x,ind,hist)
#endif
      for( y=0; y<m_h; y++ )
      {
         for( x=0; x<m_w; x++ )
         {
            ind = y*m_w+x;
            hist = dst+ind*m_hist_th_q_no;
            compute_histogram( src, y, x, hist );
         }
      }
   }
   if(m_verbosity > 0 ) {
      time(&en);
      cout<<" in "<<difftime(en,st)<<" seconds\n";
   }
}

void daisy::normalize_histograms()
{
   for( int r=0; r<g_cube_number; r++ )
   {
      float* dst = m_smoothed_gradient_layers+r*m_cube_size;

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
      for( int y=0; y<m_h; y++ )
      {
         for( int x=0; x<m_w; x++ )
         {
            float* hist = dst + (y*m_w+x)*m_hist_th_q_no;
            float norm =  l2norm( hist, m_hist_th_q_no );
            if( norm != 0.0 ) divide( hist, m_hist_th_q_no, norm);
         }
      }
   }
}

void daisy::compute_smoothed_gradient_layers()
{
   time_t st, en;
   if( m_verbosity > 0 ) {
      cout<<"[daisy] computing smoothed orientation layers...";
      time(&st);
   }

   float* prev_cube = m_smoothed_gradient_layers;
   float* cube = NULL;

   double sigma;
   for( int r=0; r<g_cube_number; r++ )
   {
      cube = m_smoothed_gradient_layers + (r+1)*m_cube_size;

      // incremental smoothing
      if( r == 0 ) sigma = m_cube_sigmas[0];
      else         sigma = sqrt( m_cube_sigmas[r]*m_cube_sigmas[r] - m_cube_sigmas[r-1]*m_cube_sigmas[r-1] );

      int fsz = filter_size(sigma);
      float* filter = new float[fsz];
      gaussian_1d(filter, fsz, sigma, 0);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
      for( int th=0; th<m_hist_th_q_no; th++ )
      {
         convolve_sym( prev_cube+th*m_layer_size, m_h, m_w, filter, fsz, cube+th*m_layer_size );
      }
      deallocate(filter);
      prev_cube = cube;
   }
   if( m_verbosity > 0 ) {
      time(&en);
      std::cout<<" in "<<difftime(en,st)<<" seconds\n";
   }

   if( m_verbosity >= 4 ) {
      for( int r=0; r<g_cube_number; r++ ) {
         float* cube=m_smoothed_gradient_layers+(r+1)*m_cube_size;
         for( int ii=0; ii<m_hist_th_q_no; ii++ ) {
            string file = "cube"+num2str(r)+"_layer"+num2str(ii)+".bin";
            cout<<"[initialize] saving "<<file<<endl;
            save_binary(file,cube+ii*m_h*m_w, m_h, m_w, 1, TYPE_FLOAT);
         }
      }
   }
   compute_histograms();
}

void daisy::compute_oriented_grid_points()
{
   m_oriented_grid_points = allocate<double>(g_grid_orientation_resolution, m_grid_point_number*2 );

   for( int i=0; i<g_grid_orientation_resolution; i++ )
   {
      double angle = -i*2.0*PI/g_grid_orientation_resolution;

      double kos = cos( angle );
      double zin = sin( angle );

      double* point_list = m_oriented_grid_points[ i ];

      for( int k=0; k<m_grid_point_number; k++ )
      {
         double y = m_grid_points[k][0];
         double x = m_grid_points[k][1];

         point_list[2*k+1] =  x*kos + y*zin; // x
         point_list[2*k  ] = -x*zin + y*kos; // y
      }
   }

   if( m_verbosity > 1 )
   {
      // if want to display the descriptor structure
      int str_w = (int)(2*m_rad+1);
      int str_size = square( str_w );

      int* structure = allocate<int>( str_size );

      for( int ori=0; ori<g_grid_orientation_resolution; ori++ )
      {
         memset( structure, 0, sizeof(int)*str_size );
         for( int reg=0; reg<m_grid_point_number; reg++ )
         {
            int y = (int)(m_oriented_grid_points[ori] [2*reg  ] + m_rad);
            int x = (int)(m_oriented_grid_points[ori] [2*reg+1] + m_rad);

            structure[y*str_w+x] = reg+1;
         }
//          display( structure, str_w, str_w, 1, 1, 2, 2, "  " );
//          wait_key();
      }
      deallocate(structure);
   }
}

/// sets a custom grid
void daisy::set_grid_points()
{
   // I should implement this
   cout<<"[daisy] set_grid_points::this is not implemented yet \n";
   exit(1);
}

void daisy::smooth_histogram(float *hist, int hsz)
{
   int i;
   float prev, temp;

   prev = hist[hsz - 1];
   for (i = 0; i < hsz; i++)
   {
      temp = hist[i];
      hist[i] = (prev + hist[i] + hist[(i + 1 == hsz) ? 0 : i + 1]) / 3.0;
      prev = temp;
   }
}

float daisy::interpolate_peak(float left, float center, float right)
{
   if( center < 0.0 )
   {
      left = -left;
      center = -center;
      right = -right;
   }
   assert(center >= left  &&  center >= right);

   float den = (left - 2.0 * center + right);

   if( den == 0 ) return 0;
   else           return 0.5*(left -right)/den;
}

int daisy::filter_size( double sigma )
{
   int fsz = (int)(5*sigma);

   // kernel size must be odd
   if( fsz%2 == 0 ) fsz++;

   // kernel size cannot be smaller than 3
   if( fsz < 3 ) fsz = 3;

   return fsz;
}

void daisy::compute_scales()
{
   cout<<"###############################################################################\n";
   cout<<"# scale detection is work-in-progress! do not use it if you're not Engin Tola #\n";
   cout<<"###############################################################################\n\n";

   int imsz = m_w * m_h;

   if( m_verbosity > 0 )
   {
      cout<<"[daisy] detecting scales...\n";
      cout<<"[daisy] k: "<<g_sigma_step<<" scale_st: "<<g_scale_st<<" scale_en:  "<<g_scale_en<<endl;
   }

   float sigma = pow( g_sigma_step, g_scale_st)*g_sigma_0;

   float* sim = blur_gaussian_2d<float,float>( m_image, m_h, m_w, sigma, filter_size(sigma), false);

   float* next_sim = NULL;

   float* max_dog = allocate<float>(imsz);

   m_scale_map = allocate<float>(imsz);

   memset( max_dog, 0, imsz*sizeof(float) );
   memset( m_scale_map, 0, imsz*sizeof(float) );

   int i;
   float sigma_prev;
   float sigma_new;
   float sigma_inc;

   sigma_prev = g_sigma_0;
   for( i=0; i<g_scale_en; i++ )
   {
      sigma_new  = pow( g_sigma_step, g_scale_st+i  ) * g_sigma_0;
      sigma_inc  = sqrt( sigma_new*sigma_new - sigma_prev*sigma_prev );
      sigma_prev = sigma_new;

      if( m_verbosity > 0 )
         cout <<"[daisy] [i = "<<i<<"/"<<g_scale_en<<"] smoothing: sigma_inc = "
              <<sigma_inc<<", sigma_new = "<<sigma_new<<", fsz : "<<filter_size(sigma_inc)<<endl;

      next_sim = blur_gaussian_2d<float,float>( sim, m_h, m_w, sigma_inc, filter_size( sigma_inc ) , false);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
      for( int p=0; p<imsz; p++ )
      {
         float dog = fabs( next_sim[p] - sim[p] );
         if( dog > max_dog[p] )
         {
            max_dog[p] = dog;
            m_scale_map[p] = i;
         }
      }
      deallocate( sim );

      sim = next_sim;
   }

   blur_gaussian_2d<float,float>( m_scale_map, m_h, m_w, 10.0, filter_size(10), true);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
   for( int q=0; q<imsz; q++ )
   {
      m_scale_map[q] = round( m_scale_map[q] );
   }

//    save( m_scale_map, m_h, m_w, "scales.dat");

   deallocate( sim );
   deallocate( max_dog );
}
void daisy::compute_orientations()
{
   cout<<"#####################################################################################\n";
   cout<<"# orientation detection is work-in-progress! do not use it if you're not Engin Tola #\n";
   cout<<"#####################################################################################\n\n";

   time_t sto, eno;
   time(&sto);

   if( m_verbosity > 0 )
      cout<<"[daisy] starting orientation computation\n";

   assert( m_image != NULL );

   int data_size = m_w*m_h;
   float* rotation_layers = layered_gradient( m_image, m_h, m_w, m_orientation_resolution );

   m_orientation_map = new int[data_size];
   memset( m_orientation_map, 0, sizeof(int)*data_size );

   int ori, max_ind;
   int ind;
   float max_val;

   int next, prev;
   float peak, angle;

   int x, y, kk;

   float* hist=NULL;

   float sigma_inc;
   float sigma_prev = 0;
   float sigma_new;

   for( int scale=0; scale<g_scale_en; scale++ )
   {
      sigma_new  = pow( g_sigma_step, scale  ) * m_rad/3.0;
      sigma_inc  = sqrt( sigma_new*sigma_new - sigma_prev*sigma_prev );
      sigma_prev = sigma_new;

      smooth_layers( rotation_layers, m_h, m_w, m_orientation_resolution, sigma_inc);

      for( y=0; y<m_h; y ++ )
      {
         hist = allocate<float>(m_orientation_resolution);

         for( x=0; x<m_w; x++ )
         {
            ind = y*m_w+x;

            if( m_scale_invariant && m_scale_map[ ind ] != scale ) continue;

            for( ori=0; ori<m_orientation_resolution; ori++ )
            {
               hist[ ori ] = rotation_layers[ori*data_size+ind];
            }

            for( kk=0; kk<6; kk++ )
               smooth_histogram( hist, m_orientation_resolution );

            max_val = -1;
            max_ind =  0;
            for( ori=0; ori<m_orientation_resolution; ori++ )
            {
               if( hist[ori] > max_val )
               {
                  max_val = hist[ori];
                  max_ind = ori;
               }
            }

            prev = max_ind-1;
            if( prev < 0 )
               prev += m_orientation_resolution;

            next = max_ind+1;
            if( next >= m_orientation_resolution )
               next -= m_orientation_resolution;

            peak = interpolate_peak(hist[prev], hist[max_ind], hist[next]);
            angle = (max_ind + peak)*360.0/m_orientation_resolution;

            int iangle = int(angle);

            if( iangle <    0 ) iangle += 360;
            if( iangle >= 360 ) iangle -= 360;


            if( !(iangle >= 0.0 && iangle < 360.0) )
            {
               angle = 0;
            }

            m_orientation_map[ ind ] = iangle;
         }
         deallocate(hist);
      }
   }
//    save( m_orientation_map, m_h, m_w, "orientations");

   deallocate( rotation_layers );
   time(&eno);

   if( m_verbosity > 0 )
      cout<<"[daisy] finished orientation computation in "<<difftime(eno,sto)<<" seconds\n";

   compute_oriented_grid_points();
}

void daisy::set_descriptor_memory( float* descriptor, long int d_size )
{
   assert( m_descriptor_memory == false );
   assert( m_h*m_w != 0 );
   assert( d_size >= compute_descriptor_memory() );

   m_dense_descriptors = descriptor;
   m_descriptor_memory = true;
}
void daisy::set_workspace_memory( float* workspace, long int w_size )
{
//    assert( m_workspace_memory == false );
   assert( m_h*m_w != 0 );
   assert( w_size >= compute_workspace_memory() );

   m_smoothed_gradient_layers = workspace;
   m_workspace_memory = true;
}
