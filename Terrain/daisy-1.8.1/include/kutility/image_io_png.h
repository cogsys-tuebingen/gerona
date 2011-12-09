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

// #define WITH_PNG

#ifdef WITH_PNG

#ifndef KUTILITY_IMAGE_IO_PNG_H
#define KUTILITY_IMAGE_IO_PNG_H

extern "C" {
#include "png.h"
}

#include "kutility/kutility.def"

namespace kutility
{
   int  load_png(const char* file_name, uchar* &body, int &h, int &w, int &ch);
   void save_png(const char* file_name, uchar* body, int height, int width, int chl);
}

typedef struct _write_png_info
{
   double gamma;
   long width;
   long height;
   time_t modtime;
   FILE *infile;
   FILE *outfile;
   void *png_ptr;
   void *info_ptr;
   uchar *image_data;
   uchar **row_pointers;
   char *title;
   char *author;
   char *desc;
   char *copyright;
   char *email;
   char *url;
   int channel_no;
   int filter;    /* command-line-filter flag, not PNG row filter! */
   // int pnmtype;
   int sample_depth;
   int interlaced;
   int have_time;
   jmp_buf jmpbuf;
   uchar bg_red;
   uchar bg_green;
   uchar bg_blue;
} write_png_info;

void wpng_cleanup(write_png_info* a);

void writepng_version_info ();
int  writepng_init         (write_png_info *png_ptr);
int  writepng_encode_image (write_png_info *png_ptr);
int  writepng_encode_row   (write_png_info *png_ptr);
int  writepng_encode_finish(write_png_info *png_ptr);
void writepng_cleanup      (write_png_info *png_ptr);

#endif

#endif
