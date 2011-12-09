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

template<class T> inline int load( ifstream& fin, T* &out, int size )
{
   int tsize;
   bool read_all = false;
   if( size <= 0 )
   {
      tsize = 100;
      read_all = true;
   }
   else
      tsize = size;

   if( out == NULL ) out = new T[tsize];

   int counter = 0;
   fin.peek();
   while( !fin.fail() )
   {
      float val;
      fin >> val;
      if( fin.fail() ) break;

      out[counter] = val;

      counter++;
      if( counter >= tsize )
      {
         if( read_all )
         {
            expand_array( out, tsize, 2*tsize );
            tsize *= 2;
         }
         else
            break;
      }
   }
   if( read_all ) expand_array( out, counter, counter );

   if( size!= -1 && size != counter )
      cout<<"WARNING: I loaded only "<<counter<<" data points instead of "<<size<<"\n";

   return counter;
}

template<class T> inline int load( string filename, T* &out, int size )
{
   ifstream fin;
   fin.open( filename.c_str() );
   if( fin.fail() )
   {
      cout<<"no such file: "<<filename<<endl;
      exit(1);
   }
   int retval = load(fin, out, size );
   fin.close();
   return retval;
}

template<class T> inline void save_binary(ofstream& fout, T* data, int h, int w, int nb, int type )
{
   fout.write((char*)&type, sizeof(int));
   fout.write((char*)&h,    sizeof(int));
   fout.write((char*)&w,    sizeof(int));
   fout.write((char*)&nb,   sizeof(int));
   fout.write((char*)data,  sizeof(T)*h*w*nb );
}
template<class T> inline int  save_binary(string filename, T* data, int h, int w, int nb, int type )
{
   ofstream fout(filename.c_str(), ofstream::binary);
   if( fout.fail() )
   {
      warning("cannot open file: ", filename);
      return 1;
   }
   save_binary( fout, data, h, w, nb, type );
   fout.close();
   return 0;
}

template<typename T> inline void save_ascii( ofstream& fout,  T* data, int h, int w, int nb, int type )
{
   fout<<type<<" "<<h<<" "<<w<<" "<<nb<<endl;
   int sz = h*w;
   for( int i=0; i<sz; i++ )
      fout<<data[i]<<" ";
   return;
}
template<typename T> inline void save_ascii( string filename, T* data, int h, int w, int nb, int type )
{
   std::ofstream fout;
   fout.open( filename.c_str(), std::ofstream::out );

   save_ascii( fout, data, h, w, nb, type );

   fout.close();
   return;
}
template<typename T> inline void load_ascii( ifstream& fin,   T* &data, int &h, int &w, int &nb )
{
   int type = 0;
   fin >> type >> h >> w >> nb;
   data = new T[h*w*nb];

   char tchar;
   int tint;
   float tfloat;
   double tdbl;

   int sz = h*w*nb;
   for( int k=0; k<sz; k++ )
   {
      if( type == TYPE_INT )
      {
         fin >> tint;
         data[k] = (int)tint;
         continue;
      }
      if( type == TYPE_FLOAT )
      {
         fin >> tfloat;
         data[k] = (float)tfloat;
         continue;
      }
      if( type == TYPE_DOUBLE )
      {
         fin >> tdbl;
         data[k] = (double)tdbl;
         continue;
      }
      if( type == TYPE_CHAR )
      {
         fin >> tchar;
         data[k] = (char)tchar;
         continue;
      }
   }
}
template<typename T> inline void load_ascii( string filename, T* &data, int &h, int &w, int &nb )
{
   ifstream fin( filename.c_str(), ifstream::in );
   if( fin.fail() ) error("cannot open file: ", filename );
   load_ascii( fin, data, h, w, nb );
   fin.close();

}

inline int load_binary(ifstream &fin, float*  &data, int &h, int &w, int &nb )
{
   int type = 0;
   fin.read((char*)&type, sizeof(int));
   fin.read((char*)&h,    sizeof(int));
   fin.read((char*)&w,    sizeof(int));
   fin.read((char*)&nb,   sizeof(int));
   if( type != TYPE_FLOAT )
   {
      fin.close();
      return 1;
   }

   data = new float[h*w*nb];
   fin.read((char*)data, sizeof(float)*h*w*nb );
   return 0;
}
inline int load_binary(ifstream &fin, int*    &data, int &h, int &w, int &nb )
{
   int type = 0;
   fin.read((char*)&type, sizeof(int));
   fin.read((char*)&h,    sizeof(int));
   fin.read((char*)&w,    sizeof(int));
   fin.read((char*)&nb,   sizeof(int));
   if( type != TYPE_INT )
   {
      fin.close();
      return 1;
   }

   data = new int[h*w*nb];
   fin.read((char*)data, sizeof(int)*h*w*nb );
   return 0;
}
inline int load_binary(ifstream &fin, double* &data, int &h, int &w, int &nb )
{
   int type = 0;
   fin.read((char*)&type, sizeof(int));
   fin.read((char*)&h,    sizeof(int));
   fin.read((char*)&w,    sizeof(int));
   fin.read((char*)&nb,   sizeof(int));
   if( type != TYPE_DOUBLE )
   {
      fin.close();
      return 1;
   }

   data = new double[h*w*nb];
   fin.read((char*)data, sizeof(double)*h*w*nb);
   return 0;
}
inline int load_binary(ifstream &fin, char*   &data, int &h, int &w, int &nb )
{
   int type = 0;
   fin.read((char*)&type, sizeof(int));
   fin.read((char*)&h,    sizeof(int));
   fin.read((char*)&w,    sizeof(int));
   fin.read((char*)&nb,   sizeof(int));
   if( type != TYPE_CHAR )
   {
      fin.close();
      return 1;
   }

   data = new char[h*w*nb];
   fin.read((char*)data, sizeof(char)*h*w*nb);
   return 0;
}

template<typename T> inline int  load_binary(string filename, T* &data, int &h, int &w, int &nb )
{
   ifstream fin( filename.c_str(), ifstream::binary );
   if( fin.fail() ) error("cannot open file: ", filename );
   int retval = load_binary( fin, data, h, w, nb );
   fin.close();
   return retval;
}

template<class T> inline void save_plain(ofstream& fout, T* data, int sz )
{
   for( int i=0; i<sz; i++ )
      fout<<data[i]<<" ";
//    fout<<"\n"; // commented by Yasir
}

template<class T> inline void save_plain(ofstream& fout, T* data, int rs, int cs )
{
   for( int r=0; r<rs; r++ )
   {
      for( int c=0; c<cs; c++ )
      {
         fout<<data[r*cs+c]<<" ";
      }
//       fout<<"\n"; // commented by Yasir
   }
}

template<class T> inline void save(string filename, T* data, int sz)
{
   std::ofstream fout;
   fout.open( filename.c_str(), std::ofstream::out );
   save_plain( fout, data, sz );
   fout.close();
   return;
}

/// saves an array in matrix format with rs x cs
template<class T> inline void save(string filename, T* data, int rs, int cs, bool append)
{
  std::ofstream fout;
  if (append)
    fout.open( filename.c_str(), std::ofstream::out|std::ofstream::app );
  else
    fout.open( filename.c_str(), std::ofstream::out );
  if (!fout)
  {
    cout << "ERROR: Unable to open file " << filename << endl;
    exit(1);
  }
  save_plain( fout, data, rs, cs );
  fout << endl;
  fout.close();
  return;
}

/// loads an array given its size and data type. supported data
/// types are int and double. "type" should be set to 0 for integer
/// and to 1 for a double data. By default it is set to double.
inline void* load_array( string filename, int size, int type )
{
   if( type > 2 )
   {
      kutility::error("load_array: unsupported type", filename);
   }

   FILE* fp = fopen(filename.c_str(),"r");

   if( fp == NULL )
   {
      kutility::error("load_array: unable to open ", filename);
   }

   double * d_data=NULL;
   float  * f_data=NULL;
   int    * n_data=NULL;

   double d;
   float  f;
   int    n;

   int scanf_ret = 0;

   if( type == 0 ) n_data = new int   [size];
   if( type == 1 ) d_data = new double[size];
   if( type == 2 ) f_data = new float [size];

   for( int i=0; i<size; i++ )
   {
      if( type == 0 ) scanf_ret = fscanf(fp," %d" ,&n);
      if( type == 1 ) scanf_ret = fscanf(fp," %lg", &d);
      if( type == 2 ) scanf_ret = fscanf(fp," %f" ,&f);

      if( scanf_ret != 1 ) break;

      if( type == 0 ) n_data[i] = n;
      if( type == 1 ) d_data[i] = d;
      if( type == 2 ) f_data[i] = f;
   }
   fclose(fp);

   if( type == 0 ) return (void *)n_data;
   if( type == 1 ) return (void *)d_data;
   if( type == 2 ) return (void *)f_data;
   return NULL;
}
