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

#ifndef KUTILITY_GENERAL_H
#define KUTILITY_GENERAL_H

#ifndef WIN32
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

#if defined(WIN32)
#define strncasecmp _strnicmp
#pragma warning( disable : 4244 4305 )
#endif

#include "kutility/interaction.h"
#include "string.h"
namespace kutility
{
   struct kpoint
   {
      int x;
      int y;
   };


   template<class T>
   class rectangle
   {
   public:
      T lx, ux, ly, uy;
      T dx, dy;
      rectangle(T xl, T xu, T yl, T yu) { lx=xl; ux=xu; ly=yl; uy=yu; dx=ux-lx; dy=uy-ly; };
      rectangle()                       { lx = ux = ly = uy = dx = dy = 0; };
   };

   inline bool is_line( const char* str1, const char* str2 )
   {
      return !strncasecmp(str1, str2, strlen(str2));
   }

   /// checks if the number x is between lx - ux interval.
   /// the equality is checked depending on the value of le and ue parameters.
   /// if le=1 => lx<=x is checked else lx<x is checked
   /// if ue=1 => x<=ux is checked else x<ux is checked
   /// by default x is searched inside of [lx,ux)
   template<class T1, class T2, class T3> inline
   bool is_inside(T1 x, T2 lx, T3 ux, bool le=true, bool ue=false)
   {
      if( ( ((lx<x)&&(!le)) || ((lx<=x)&&le) ) && ( ((x<ux)&&(!ue)) || ((x<=ux)&&ue) )    )
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   /// checks if the number x is between lx - ux and/or y is between ly - uy interval.
   /// If the number is inside, then function returns true, else it returns false.
   /// the equality is checked depending on the value of le and ue parameters.
   /// if le=1 => lx<=x is checked else lx<x is checked
   /// if ue=1 => x<=ux is checked else x<ux is checked
   /// by default x is searched inside of [lx,ux).
   /// the same equality check is applied to the y variable as well.
   /// If the 'oper' is set '&' both of the numbers must be within the interval to return true
   /// But if the 'oper' is set to '|' then only one of them being true is sufficient.
   template<class T1, class T2, class T3> inline
   bool is_inside(T1 x, T2 lx, T3 ux, T1 y, T2 ly, T3 uy, bool le=true, bool ue=false, char oper='&')
   {
      switch( oper )
      {
      case '|':
         if( is_inside(x,lx,ux,le,ue) || is_inside(y,ly,uy,le,ue) )
            return true;
         return false;

      default:
         if( is_inside(x,lx,ux,le,ue) && is_inside(y,ly,uy,le,ue) )
            return true;
         return false;
      }
   }

   /// checks if the number x is between lx - ux and/or y is between ly - uy interval.
   /// If the number is inside, then function returns true, else it returns false.
   /// the equality is checked depending on the value of le and ue parameters.
   /// if le=1 => lx<=x is checked else lx<x is checked
   /// if ue=1 => x<=ux is checked else x<ux is checked
   /// by default x is searched inside of [lx,ux).
   /// the same equality check is applied to the y variable as well.
   /// If the 'oper' is set '&' both of the numbers must be within the interval to return true
   /// But if the 'oper' is set to '|' then only one of them being true is sufficient.
   template<class T1, class T2> inline
   bool is_inside(T1 x, T1 y, rectangle<T2> roi, bool le=true, bool ue=false, char oper='&')
   {
      switch( oper )
      {
      case '|':
         if( is_inside(x,roi.lx,roi.ux,le,ue) || is_inside(y,roi.ly,roi.uy,le,ue) )
            return true;
         return false;

      default:
         if( is_inside(x,roi.lx,roi.ux,le,ue) && is_inside(y,roi.ly,roi.uy,le,ue) )
            return true;
         return false;
      }
   }


   /// checks if the number x is outside lx - ux interval
   /// the equality is checked depending on the value of le and ue parameters.
   /// if le=1 => lx>x is checked else lx>=x is checked
   /// if ue=1 => x>ux is checked else x>=ux is checked
   /// by default is x is searched outside of [lx,ux)
   template<class T1, class T2, class T3> inline
   bool is_outside(T1 x, T2 lx, T3 ux, bool le=true, bool ue=false)
   {
      return !(is_inside(x,lx,ux,le,ue));
   }

   /// checks if the numbers x and y is outside their intervals.
   /// The equality is checked depending on the value of le and ue parameters.
   /// If le=1 => lx>x is checked else lx>=x is checked
   /// If ue=1 => x>ux is checked else x>=ux is checked
   /// By default is x is searched outside of [lx,ux) (Similarly for y)
   /// By default, 'oper' is set to OR. If one of them is outside it returns
   /// true otherwise false.
   template<class T1, class T2, class T3> inline
   bool is_outside(T1 x, T2 lx, T3 ux, T1 y, T2 ly, T3 uy, bool le=true, bool ue=false, char oper='|')
   {
      switch( oper )
      {
      case '&':
         if( is_outside(x,lx,ux,le,ue) && is_outside(y,ly,uy,le,ue) )
            return true;
         return false;
      default:
         if( is_outside(x,lx,ux,le,ue) || is_outside(y,ly,uy,le,ue) )
            return true;
         return false;
      }
   }

   /// checks if the numbers x and y is outside their intervals.
   /// The equality is checked depending on the value of le and ue parameters.
   /// If le=1 => lx>x is checked else lx>=x is checked
   /// If ue=1 => x>ux is checked else x>=ux is checked
   /// By default is x is searched outside of [lx,ux) (Similarly for y)
   /// By default, 'oper' is set to OR. If one of them is outside it returns
   /// true otherwise false.
   template<class T1, class T2> inline
   bool is_outside(T1 x, T1 y, rectangle<T2> roi, bool le=true, bool ue=false, char oper='|')
   {
      switch( oper )
      {
      case '&':
         if( is_outside(x,roi.lx,roi.ux,le,ue) && is_outside(y,roi.ly,roi.uy,le,ue) )
            return true;
         return false;
      default:
         if( is_outside(x,roi.lx,roi.ux,le,ue) || is_outside(y,roi.ly,roi.uy,le,ue) )
            return true;
         return false;
      }
   }

   /// waits for an input from the console.
   inline void wait_key()
   {
      char c;
      std::cout<<"\nkey in an input to continue ";
      std::cin>>c;
   }

   /// increases the size of the array from size to nsize. does not make any sanity check.
   template<class T> inline
   void expand_array( T* &array, int size, int nsize )
   {
      T* out = new T[nsize];
      memcpy( out, array, size*sizeof(T) );
      delete []array;
      array = out;
   }

   /// allocates a memory of size sz and returns a pointer to the array
   template<class T> inline
   T* allocate(const int sz)
   {
      T* array = new T[sz];
      return array;
   }

   /// allocates a memory of size ysz x xsz and returns a double pointer to it
   template<class T> inline
   T** allocate(const int ysz, const int xsz)
   {
      T** mat = new T*[ysz];
      int i;

      for(i=0; i<ysz; i++ )
         mat[i] = new T[xsz];
      // allocate<T>(xsz);

      return mat;
   }

   /// deallocates the memory and sets the pointer to null.
   template<class T> inline
   void deallocate(T* &array)
   {
      delete[] array;
      array = NULL;
   }

   /// deallocates the memory and sets the pointer to null.
   template<class T> inline
   void deallocate(T** &mat, int ysz)
   {
      if( mat == NULL ) return;

      for(int i=0; i<ysz; i++)
         deallocate(mat[i]);

      delete[] mat;
      mat = NULL;
   }

   /// makes a clone of the source array.
   template<class T> inline
   T* clone( T* src, int sz)
   {
      T* dst = allocate<T>(sz);
      memcpy( dst, src, sizeof(T)*sz);
      return dst;
   }

   /// makes a clone of the source matrix.
   template<class T> inline
   T** clone( T** src, int r, int c)
   {
      T** dst = allocate<T>(r,c);

      for( int i=0; i<r; i++ )
         memcpy( dst[i], src[i], sizeof(T)*c);
      return dst;
   }

   /// makes a copy of the source array.
   template<class T> inline
   void copy( T* dst, T* src, int sz)
   {
      memcpy( dst, src, sizeof(T)*sz);
   }

   /// makes a copy of the source matrix.
   template<class T> inline
   void copy( T** dst, T** src, int ysz, int xsz)
   {
      int y;

      for( y=0; y<ysz; y++ )
         memcpy( dst[y], src[y], sizeof(T)*xsz);
   }

   /// casts a type T2 array into a type T1 array.
   template<class T1, class T2> inline
   T1* type_cast(T2* data, int sz)
   {
      T1* out = new T1[sz];

      for( int i=0; i<sz; i++ )
         out[i] = (T1)data[i];

      return out;
   }

   char* strrev(char* szT);

   /// converts a number to an array.
   char* itoa(int value, char* str, int radix);

   /// converts an integer into a string.
   inline std::string num2str( int n )
   {
      char buf[10];
      itoa(n,buf,10);
      std::string retval = buf;
      return retval;
   }

   /// initializes the array arr with value=val
   template<class T> inline
   void initialize(T* &arr, int sz, unsigned char val=0)
   {
      if( arr == NULL ) error("you should allocate memory first");
      for( int i=0; i<sz; i++ )
         arr[i] = val;
   }

   /// initializes the matrix mat with value=val
   template<class T> inline
   void initialize(T** &mat, int ysz, int xsz, unsigned char val=0)
   {
      if( mat == NULL ) error("you should allocate memory first");

      for( int i=0; i<ysz; i++ )
         initialize(mat[i], xsz, val);
   }

   // template<class T> inline
   // T precision(T num, int prec)
   // {
      // double mult = pow(10.0,prec);

      // T tmp = (T)floor(mult*num);
      // tmp /= mult;
      // return tmp;
   // }

   // template<class T>
   // T* precision(T* arr, int sz, int prec, bool in_place=true)
   // {
      // T* out;

      // if( in_place ) out = arr;
      // else           out = new T[sz];

      // double q = pow(10.0, prec);

      // for(int i=0; i<sz; i++)
         // out[i] = precision( arr[i], prec );

      // return out;
   // }

   // template<class T>
   // T** precision(T** arr, int r, int c, int prec, bool in_place=true)
   // {
      // T** out;

      // if( in_place ) out = arr;
      // else           out = allocate<T>(r,c);

      // double q = pow(10.0, prec);

      // int rr,cc;

      // for( rr=0; rr<r; rr++ )
         // for( cc=0; cc<c; cc++ )
            // out[rr][cc] = precision(arr[rr][cc], prec);

      // return out;
   // }

   template<typename T> inline
   void min_max( T* data, int sz, T &mn, T &mx )
   {
      mn = data[0];
      mx = data[0];

      for( int k=1; k<sz; k++ )
      {
         if( mn > data[k] ) mn = data[k];
         if( mx < data[k] ) mx = data[k];
      }
   }

   /// finds the minimum and returns the value and its index.
   /// index is returned in the xmn parameter.
   template<class T> inline
   T min(T* data, int sz, int &xmn)
   {
      T minVal=data[0];
      xmn = 0;

      for(int i=1; i<sz; i++ )
      {
         if( minVal > data[i] )
         {
            minVal   = data[i];
            xmn = i;
         }
      }
      return minVal;
   }

   /// finds the minimum and returns the value and its index.
   /// index is returned in the xmn & ymn parameters.
   template<class T> inline
   T min(T** data, int ysz, int xsz, int &ymn, int &xmn)
   {
      T minVal = data[0][0];
      xmn = 0;
      ymn = 0;
      int minx;

      T mn;

      for( int y=0; y<ysz; y++ )
      {
         mn = min(data[y], xsz, minx);

         if( mn < minVal )
         {
            minVal = mn;
            ymn = y;
            xmn = minx;
         }
      }
      return minVal;
   }

   /// finds the maximum and returns the value and its index.
   /// index is returned in the xmx parameter.
   template<class T> inline
   T max(T* data, int sz, int &xmx)
   {
      T maxVal=data[0];
      xmx = 0;

      for(int i=1; i<sz; i++ )
      {
         if( maxVal < data[i] )
         {
            maxVal   = data[i];
            xmx = i;
         }
      }
      return maxVal;
   }

   /// finds the maximum and returns the value and its index.
   /// index is returned in the xmx and ymx parameters.
   template<class T> inline
   T max(T** data, int ysz, int xsz, int &ymx, int &xmx)
   {
      T maxVal = data[0][0];
      xmx = 0;
      ymx = 0;

      int maxx;
      T mx;

      for( int y=0; y<ysz; y++ )
      {
         mx = max(data[y], xsz, maxx);

         if( mx > maxVal )
         {
            maxVal = mx;
            ymx = y;
            xmx = maxx;
         }
      }
      return maxVal;
   }

   /// compares two arrays and returns the maximum elements
   /// if in_place = true returns the result in the first array
   template<class T> inline
   T* max( T* arr_0, T* arr_1, size_t sz, bool in_place=false )
   {
      T* result = NULL;
      if( in_place )
         result = arr_0;
      else
         result = allocate<T>(sz);

      T* p0 = arr_0;
      T* p1 = arr_1;
      T* r  = result;

      for( int i=0; i<sz; i++ )
      {
         if( *p0 > *p1 ) *r = *p0;
         else            *r = *p1;

         p0++;
         p1++;
         r++;
      }
      return result;
   }

   /// finds the interval index the number is in between.
   /// "equality" specifies the use of = or not.
   /// equality = 0 -> NN <  <  |
   /// equality = 1 -> NE <  <= |
   /// equality = 2 -> EN <= <  |
   /// equality = 3 -> EE <= <= |
   template<class T> inline
   int find_interval( T number, T** list, int lsz, int equality )
   {

      for( int i=0; i<lsz; i++ )
      {
         switch(equality)
         {
         case 0: // NN
            if( is_inside( number, list[i][0], list[i][1], 0, 0) )
               return i;
            break;
         case 1: // NE
            if( is_inside( number, list[i][0], list[i][1], 0, 1) )
               return i;
            break;
         case 2: // EN
            if( is_inside( number, list[i][0], list[i][1], 1, 0) )
               return i;
            break;
         case 3: // EE
            if( is_inside( number, list[i][0], list[i][1], 1, 1) )
               return i;
            break;
         default:
            return -1;
            break;
         }
      }
      return -1;
   }


   /// finds the interval index the number is in between.
   /// "equality" specifies the use of = or not.
   /// equality = 0 -> NN <  <  |
   /// equality = 1 -> NE <  <= |
   /// equality = 2 -> EN <= <  |
   /// equality = 3 -> EE <= <= |
   template<class T> inline
   int find_interval( T nx, T ny, T** list, int lsz, int equality )
   {
      for( int i=0; i<lsz; i++ )
      {
         switch(equality)
         {
         case 0: // NN
            if( is_inside( nx, list[i][0], list[i][1], ny, list[i][2], list[i][3], 0, 0) )
               return i;
            break;
         case 1: // NE
            if( is_inside( nx, list[i][0], list[i][1], ny, list[i][2], list[i][3], 0, 1) )
               return i;
            break;
         case 2: // EN
            if( is_inside( nx, list[i][0], list[i][1], ny, list[i][2], list[i][3], 1, 0) )
               return i;
            break;
         case 3: // EE
            if( is_inside( nx, list[i][0], list[i][1], ny, list[i][2], list[i][3], 1, 1) )
               return i;
            break;
         default:
            return -1;
            break;
         }
      }
      return -1;
   }

   inline bool is_digit( char c )
   {
      for( int i=0; i<10; i++ )
         if( c == num2str(i)[0] )
            return true;
      return false;
   }

   inline bool is_number( std::string str )
   {
      int len=str.length();

      for( int i=0; i<len; i++)
      {
         if( is_digit(str[i]) || str[i] == '.' || str[i] == '-' )
            continue;
         else
            return false;
      }
      return true;
   }

   inline bool is_positive_number( std::string str )
   {
      if( !is_number( str ) ) return false;

      float number = atof( str.c_str() );

      if( number > 0.0 ) return true;

      return false;
   }

   inline bool is_negative_number( std::string str )
   {
      if( !is_number( str ) ) return false;

      float number = atof( str.c_str() );

      if( number < 0.0 ) return true;

      return false;
   }

   inline bool is_integer( std::string str )
   {
      int len=str.length();

      for( int i=0; i<len; i++)
      {
         if( is_digit(str[i]) || str[i] == '-' )
            continue;
         else
            return false;
      }
      return true;
   }

   inline bool is_positive_integer( std::string str )
   {
      if( !is_integer( str ) ) return false;

      int number = atoi( str.c_str() );

      if( number > 0 ) return true;

      return false;
   }

   inline bool is_negative_integer( std::string str )
   {
      if( !is_integer( str ) ) return false;

      int number = atoi( str.c_str() );

      if( number < 0 ) return true;

      return false;
   }

   inline void set_integer( int &location, std::string str, std::string param_name="")
   {
      if( !is_integer( str ) )
      {
         std::string errout = param_name + " should be an integer. but it is " + str;
         std::cout<<errout<<std::endl;
         exit(1);
      }
      location = atoi( str.c_str() );
   }

   inline void set_positive_integer( int &location, std::string str, std::string param_name="")
   {
      if( !is_positive_integer( str ) )
      {
         std::string errout;
         if( param_name == "" )
            errout = "parameter should be a postive integer. but it is " + str;
         else
            errout = param_name + " should be a postive integer. but it is " + str;

         std::cout<<errout<<std::endl;
         exit(1);
      }
      location = atoi( str.c_str() );
   }

   /// returns true if file exists
   inline bool does_file_exists( std::string str )
   {
      std::ifstream outfile;
      outfile.open(str.c_str());
      if( outfile.is_open() )
      {
         outfile.close();
         return true;
      }
      else
      {
         outfile.close();
         return false;
      }
   }

   inline bool check_file( string filename )
   {
      std::ifstream fin;
      fin.open(filename.c_str());
      if( fin.fail() ) return false;
      fin.close();
      return true;
   }

   template<typename T> inline
   void create_file( string filename, long int file_size )
   {
      FILE* pFile = fopen( filename.c_str() , "w+" );
      T* buffer = new T[file_size];
      fwrite(buffer , sizeof(T) , file_size , pFile );
      delete []buffer;
      fclose(pFile);
   }

#if !defined(WIN32)
   /// to deallocate call munmap( (void*)mapped_file, size * sizeof(T) )
   template<typename T> inline
   void map_memory_file(string memory_file, long int size, T* &mapped_file)
   {
      int fildes = open64(memory_file.c_str(), O_RDWR);

      if(fildes == -1) //The file does not exist
      {
         create_file<T>(memory_file,size);
         fildes = open64(memory_file.c_str(), O_RDWR);
      }

      void* file = mmap64(0, size*sizeof(T), PROT_READ|PROT_WRITE, MAP_SHARED, fildes, 0);

      if(file == MAP_FAILED)
      {
         error("file cannot be mapped");
      }

      mapped_file = (T*)file;
   }
#endif

   inline string get_file_format( string file )
   {
      size_t found = file.find_last_of(".");
      string file_format = file.substr(found+1);
      return file_format;
   }

   inline string get_file_name( string file )
   {
      size_t found = file.find_last_of("/\\");
      string file_name = file.substr(found+1);
      return file_name;
   }

   inline string get_folder_name( string file )
   {
      size_t found = file.find_last_of("/\\");
      string folder_name = file.substr(0,found);
      return folder_name;
   }

   inline string get_file_root( string file )
   {
      string file_name = get_file_name(file);
      size_t found = file_name.find_last_of(".");
      string file_root = file_name.substr(0,found);
      return file_root;
   }
}

#endif
