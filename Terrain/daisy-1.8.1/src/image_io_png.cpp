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

#define WITH_PNG

#ifdef WITH_PNG

#include "kutility/image_io_png.h"
#include <png.h>

#ifndef png_jmpbuf
#  define png_jmpbuf(png_ptr) ((png_ptr)->jmpbuf)
#endif

namespace kutility
{
   int load_png(const char* file_name, uchar* &body, int &h, int &w, int &ch)
   {
      png_structp png_ptr;
      png_infop info_ptr;
      unsigned int sig_read = 0;
      FILE *fp;

      if ((fp = fopen(file_name, "rb")) == NULL)
         return 1;

      png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
      if (png_ptr == NULL)
      {
         fclose(fp);
         return 1;
      }

      /* Allocate/initialize the memory for image information.  REQUIRED. */
      info_ptr = png_create_info_struct(png_ptr);
      if (info_ptr == NULL)
      {
         fclose(fp);
         png_destroy_read_struct(&png_ptr, png_infopp_NULL, png_infopp_NULL);
         return 1;
      }

      if (setjmp(png_jmpbuf(png_ptr)))
      {
         /* Free all of the memory associated with the png_ptr and info_ptr */
         png_destroy_read_struct(&png_ptr, &info_ptr, png_infopp_NULL);
         fclose(fp);
         /* If we get here, we had a problem reading the file */
         return 1;
      }

      /* One of the following I/O initialization methods is REQUIRED */
      /* Set up the input control if you are using standard C streams */
      png_init_io(png_ptr, fp);

      /* If we have already read some of the signature */
      png_set_sig_bytes(png_ptr, sig_read);

      png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, png_voidp_NULL);
      /* At this point you have read the entire image */

      png_bytep* row_pointers = png_get_rows(png_ptr, info_ptr);
      h  = png_get_image_height(png_ptr, info_ptr);
      w  = png_get_image_width(png_ptr, info_ptr);
      ch = png_get_channels(png_ptr, info_ptr);

      int rb = png_get_rowbytes(png_ptr, info_ptr);

      body = new uchar[h*rb];
      for( int y=0; y<h; y++ )
      {
         memcpy( body+y*rb, row_pointers[y], rb);
      }

      /* clean up after the read, and free any memory allocated - REQUIRED */
      png_destroy_read_struct(&png_ptr, &info_ptr, png_infopp_NULL);

      /* close the file */
      fclose(fp);

      /* that's it */
      return 0;
   }

   void save_png(const char* file_name, uchar* body, int height, int width, int chl)
   {
      write_png_info wpng_info;   /* lone global */

      wpng_info.infile = NULL;
      wpng_info.outfile = NULL;
      wpng_info.image_data = NULL;
      wpng_info.row_pointers = NULL;
      wpng_info.filter = false;
      wpng_info.interlaced = false;
      wpng_info.have_time = false;
      wpng_info.gamma = 0.0;
      wpng_info.channel_no = chl;

      ulong rowbytes;
      int rc;
      int error = 0;

      wpng_info.width = width;
      wpng_info.height = height;
      wpng_info.outfile = fopen(file_name,"wb");
      wpng_info.sample_depth = 8;

      if ((rc = writepng_init(&wpng_info)) != 0) {
         switch (rc) {
         case 2:
            fprintf(stderr, "libpng initialization problem (longjmp)\n");
            break;
         case 4:
            fprintf(stderr, "insufficient memory\n");
            break;
         case 11:
            fprintf(stderr, "internal logic error (unexpected PNM type)\n");
            break;
         default:
            fprintf(stderr, "unknown writepng_init() error\n");
            break;
         }
         exit(rc);
      }

      if( chl == 1 )
         rowbytes = wpng_info.width;
      else // if( ch == 3 )
         rowbytes = wpng_info.width * 3;

      // fprintf(stderr, "Encoding image data...\n");
      // fflush(stderr);

      error = 0;

      for(long j = 0; j < wpng_info.height; j++)
      {
         wpng_info.image_data = body+j*rowbytes;
         if( writepng_encode_row(&wpng_info) != 0 )
         {
            fprintf(stderr, "libpng problem (longjmp) while writing row %ld\n", j);
            ++error;
            break;
         }
      }
      if (error) {
         writepng_cleanup(&wpng_info);
         wpng_cleanup(&wpng_info);
         exit(2);
      }
      if (writepng_encode_finish(&wpng_info) != 0) {
         fprintf(stderr, "error on final libpng call\n");
         writepng_cleanup(&wpng_info);
         wpng_cleanup(&wpng_info);
         exit(2);
      }
   }

}

static void writepng_error_handler(png_structp png_ptr, png_const_charp msg);

void writepng_version_info(void)
{
   fprintf(stderr, "   Compiled with libpng %s; using libpng %s.\n",
           PNG_LIBPNG_VER_STRING, png_libpng_ver);
   fprintf(stderr, "   Compiled with zlib %s; using zlib %s.\n",
           ZLIB_VERSION, zlib_version);
}

/* returns 0 for success, 2 for libpng problem, 4 for out of memory, 11 for
 *  unexpected pnmtype; note that outfile might be stdout */
int writepng_init(write_png_info *mainprog_ptr)
{
   png_structp  png_ptr;       /* note:  temporary variables! */
   png_infop  info_ptr;
   int color_type, interlace_type;


   /* could also replace libpng warning-handler (final NULL), but no need: */

   png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, mainprog_ptr,
                                     writepng_error_handler, NULL);
   if (!png_ptr)
      return 4;   /* out of memory */

   info_ptr = png_create_info_struct(png_ptr);
   if (!info_ptr) {
      png_destroy_write_struct(&png_ptr, NULL);
      return 4;   /* out of memory */
   }


   /* setjmp() must be called in every function that calls a PNG-writing
    * libpng function, unless an alternate error handler was installed--
    * but compatible error handlers must either use longjmp() themselves
    * (as in this program) or exit immediately, so here we go: */

   if (setjmp(mainprog_ptr->jmpbuf)) {
      png_destroy_write_struct(&png_ptr, &info_ptr);
      return 2;
   }


   /* make sure outfile is (re)opened in BINARY mode */

   png_init_io(png_ptr, mainprog_ptr->outfile);


   /* set the compression levels--in general, always want to leave filtering
    * turned on (except for palette images) and allow all of the filters,
    * which is the default; want 32K zlib window, unless entire image buffer
    * is 16K or smaller (unknown here)--also the default; usually want max
    * compression (NOT the default); and remaining compression flags should
    * be left alone */

   png_set_compression_level(png_ptr, Z_BEST_COMPRESSION);
/*
  >> this is default for no filtering; Z_FILTERED is default otherwise:
  png_set_compression_strategy(png_ptr, Z_DEFAULT_STRATEGY);
  >> these are all defaults:
  png_set_compression_mem_level(png_ptr, 8);
  png_set_compression_window_bits(png_ptr, 15);
  png_set_compression_method(png_ptr, 8);
*/


   /* set the image parameters appropriately */

   if (mainprog_ptr->channel_no == 1 )
      color_type = PNG_COLOR_TYPE_GRAY;
   else if (mainprog_ptr->channel_no == 3)
      color_type = PNG_COLOR_TYPE_RGB;
   else {
      png_destroy_write_struct(&png_ptr, &info_ptr);
      return 11;
   }

   interlace_type = mainprog_ptr->interlaced? PNG_INTERLACE_ADAM7 :
      PNG_INTERLACE_NONE;

   png_set_IHDR(png_ptr, info_ptr, mainprog_ptr->width, mainprog_ptr->height,
                mainprog_ptr->sample_depth, color_type, interlace_type,
                PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

   if (mainprog_ptr->gamma > 0.0)
      png_set_gAMA(png_ptr, info_ptr, mainprog_ptr->gamma);

   if (mainprog_ptr->have_time) {
      png_time  modtime;

      png_convert_from_time_t(&modtime, mainprog_ptr->modtime);
      png_set_tIME(png_ptr, info_ptr, &modtime);
   }


   /* write all chunks up to (but not including) first IDAT */
   png_write_info(png_ptr, info_ptr);


   /* if we wanted to write any more text info *after* the image data, we
    * would set up text struct(s) here and call png_set_text() again, with
    * just the new data; png_set_tIME() could also go here, but it would
    * have no effect since we already called it above (only one tIME chunk
    * allowed) */


   /* set up the transformations:  for now, just pack low-bit-depth pixels
    * into bytes (one, two or four pixels per byte) */

   png_set_packing(png_ptr);
/*  png_set_shift(png_ptr, &sig_bit);  to scale low-bit-depth values */


   /* make sure we save our pointers for use in writepng_encode_image() */

   mainprog_ptr->png_ptr = png_ptr;
   mainprog_ptr->info_ptr = info_ptr;


   /* OK, that's all we need to do for now; return happy */

   return 0;
}

/* returns 0 for success, 2 for libpng (longjmp) problem */
int writepng_encode_image(write_png_info *mainprog_ptr)
{
   png_structp png_ptr = (png_structp)mainprog_ptr->png_ptr;
   png_infop info_ptr = (png_infop)mainprog_ptr->info_ptr;


   /* as always, setjmp() must be called in every function that calls a
    * PNG-writing libpng function */

   if (setjmp(mainprog_ptr->jmpbuf)) {
      png_destroy_write_struct(&png_ptr, &info_ptr);
      mainprog_ptr->png_ptr = NULL;
      mainprog_ptr->info_ptr = NULL;
      return 2;
   }


   /* and now we just write the whole image; libpng takes care of interlacing
    * for us */

   png_write_image(png_ptr, mainprog_ptr->row_pointers);


   /* since that's it, we also close out the end of the PNG file now--if we
    * had any text or time info to write after the IDATs, second argument
    * would be info_ptr, but we optimize slightly by sending NULL pointer: */

   png_write_end(png_ptr, NULL);

   return 0;
}

/* returns 0 if succeeds, 2 if libpng problem */
int writepng_encode_row(write_png_info *mainprog_ptr)  /* NON-interlaced only! */
{
   png_structp png_ptr = (png_structp)mainprog_ptr->png_ptr;
   png_infop info_ptr = (png_infop)mainprog_ptr->info_ptr;


   /* as always, setjmp() must be called in every function that calls a
    * PNG-writing libpng function */

   if (setjmp(mainprog_ptr->jmpbuf)) {
      png_destroy_write_struct(&png_ptr, &info_ptr);
      mainprog_ptr->png_ptr = NULL;
      mainprog_ptr->info_ptr = NULL;
      return 2;
   }


   /* image_data points at our one row of image data */

   png_write_row(png_ptr, mainprog_ptr->image_data);

   return 0;
}

/* returns 0 if succeeds, 2 if libpng problem */
int writepng_encode_finish(write_png_info *mainprog_ptr)   /* NON-interlaced! */
{
   png_structp png_ptr = (png_structp)mainprog_ptr->png_ptr;
   png_infop info_ptr = (png_infop)mainprog_ptr->info_ptr;


   /* as always, setjmp() must be called in every function that calls a
    * PNG-writing libpng function */

   if (setjmp(mainprog_ptr->jmpbuf)) {
      png_destroy_write_struct(&png_ptr, &info_ptr);
      mainprog_ptr->png_ptr = NULL;
      mainprog_ptr->info_ptr = NULL;
      return 2;
   }


   /* close out PNG file; if we had any text or time info to write after
    * the IDATs, second argument would be info_ptr: */

   png_write_end(png_ptr, NULL);

   return 0;
}

void writepng_cleanup(write_png_info *mainprog_ptr)
{
   png_structp png_ptr = (png_structp)mainprog_ptr->png_ptr;
   png_infop info_ptr = (png_infop)mainprog_ptr->info_ptr;

   if (png_ptr && info_ptr)
      png_destroy_write_struct(&png_ptr, &info_ptr);
}

static void writepng_error_handler(png_structp png_ptr, png_const_charp msg)
{
   write_png_info  *mainprog_ptr;

   /* This function, aside from the extra step of retrieving the "error
    * pointer" (below) and the fact that it exists within the application
    * rather than within libpng, is essentially identical to libpng's
    * default error handler.  The second point is critical:  since both
    * setjmp() and longjmp() are called from the same code, they are
    * guaranteed to have compatible notions of how big a jmp_buf is,
    * regardless of whether _BSD_SOURCE or anything else has (or has not)
    * been defined. */

   fprintf(stderr, "writepng libpng error: %s\n", msg);
   fflush(stderr);

   mainprog_ptr = (write_png_info*)png_get_error_ptr(png_ptr);
   if (mainprog_ptr == NULL) {         /* we are completely hosed now */
      fprintf(stderr,
              "writepng severe error:  jmpbuf not recoverable; terminating.\n");
      fflush(stderr);
      exit(99);
   }

   longjmp(mainprog_ptr->jmpbuf, 1);
}

void wpng_cleanup(write_png_info* a)
{
   if (a->outfile) {
      fclose(a->outfile);
      a->outfile = NULL;
   }

   if (a->infile) {
      fclose(a->infile);
      a->infile = NULL;
   }

   if (a->row_pointers) {
      free(a->row_pointers);
      a->row_pointers = NULL;
   }
}

#endif
