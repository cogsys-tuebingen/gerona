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

#include "kutility/image_io_pnm.h"
#define PNM_BUFFER_SIZE 256
// using namespace std;

namespace kutility
{
   void read_packed(unsigned char *data, int size, std::ifstream &f)
   {
      unsigned char c = 0;

      int bitshift = -1;
      for (int pos = 0; pos < size; pos++) {
         if (bitshift == -1) {
            c = f.get();
            bitshift = 7;
         }
         data[pos] = (c >> bitshift) & 1;
         bitshift--;
      }
   }
   void write_packed(unsigned char *data, int size, std::ofstream &f)
   {
      unsigned char c = 0;

      int bitshift = 7;
      for (int pos = 0; pos < size; pos++) {
         c = c | (data[pos] << bitshift);
         bitshift--;
         if ((bitshift == -1) || (pos == size-1)) {
            f.put(c);
            bitshift = 7;
            c = 0;
         }
      }
   }
   void pnm_read(std::ifstream &file, char *buf)
   {
      char doc[PNM_BUFFER_SIZE];
      char c;

      file >> c;
      while (c == '#') {
         file.getline(doc, PNM_BUFFER_SIZE);
         file >> c;
      }
      file.putback(c);

      file.width(PNM_BUFFER_SIZE);
      file >> buf;
      file.ignore();
   }
   void get_size_ppm(const char *name, int &height, int &width)
   {
      char buf[PNM_BUFFER_SIZE];
      //char doc[PNM_BUFFER_SIZE]
      // read header
      std::ifstream file(name, std::ios::in | std::ios::binary);
      pnm_read(file, buf);
      if (strncmp(buf, "P6", 2))
      {
         printf("type mismatch\n");
         exit(1);
      }

      pnm_read(file, buf);
      width = atoi(buf);

      pnm_read(file, buf);
      height = atoi(buf);

      file.close();
      return;
   }

   void load_pbm(const char* name, uchar* &im, int &height, int &width)
   {
      char buf[PNM_BUFFER_SIZE];

      /* read header */
      std::ifstream file(name, std::ios::in | std::ios::binary);
      pnm_read(file, buf);
      if (strncmp(buf, "P4", 2))
      {
         printf("type mismatch\n");
         exit(1);
      }

      pnm_read(file, buf);
      width = atoi(buf);

      pnm_read(file, buf);
      height = atoi(buf);

      /* read data */
      if( im != NULL) delete[]im;
      im = new uchar[width*height];
      for (int i = 0; i < height; i++)
         read_packed(im+(width*i), width, file);
   }
   void load_pgm(const char* name, uchar* &im, int &height, int& width)
   {
      char buf[PNM_BUFFER_SIZE];

      /* read header */
      std::ifstream file(name, std::ios::in | std::ios::binary);
      pnm_read(file, buf);
      if (strncmp(buf, "P5", 2))
      {
         printf("type mismatch\n");
         exit(1);
      }

      pnm_read(file, buf);
      width = atoi(buf);

      pnm_read(file, buf);
      height = atoi(buf);

      pnm_read(file, buf);
      if (atoi(buf) > UCHAR_MAX)
      {
         printf("type mismatch\n");
         exit(1);
      }

      /* read data */
      if( im != NULL ) delete[] im;
      im = new uchar[width*height];
      file.read( (char *)im, width * height * sizeof(uchar));
   }

   void load_ppm(const char* name, uchar* &im, int &height, int &width)
   {
      char buf[PNM_BUFFER_SIZE];
      //char doc[PNM_BUFFER_SIZE]
      std::ifstream file(name, std::ios::in | std::ios::binary);
      pnm_read(file, buf);
      if (strncmp(buf, "P6", 2))
      {
         printf("type mismatch\n");;
         exit(1);
      }
      pnm_read(file, buf);
      width = atoi(buf);

      pnm_read(file, buf);
      height = atoi(buf);

      pnm_read(file, buf);
      if (atoi(buf) > UCHAR_MAX)
      {
         printf("type mismatch\n");;
         exit(1);
      }

      /* read data */
      if( im != NULL ) delete[] im;
      im = new uchar[width*height*3];
      file.read((char *)im, width * height * 3 * sizeof(uchar));
   }

   void save_pbm(const char* name, uchar* im, int height, int width )
   {
      std::ofstream file(name, std::ios::out | std::ios::binary);

      file << "P4\n" << width << " " << height << "\n";
      for (int i = 0; i < height; i++)
         write_packed(im+(width*i), width, file);
   }


}
