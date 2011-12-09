#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <cstring>
#include <cmath>
#include <cassert>
#include <cstdlib>
#include <fstream>

#include "CCH.h"

using namespace std;

const float TWO_PI = 2.0f * M_PI;

float* cch_desc( int* img, int imgW, int imgH, float*& desc, int& descSize )
{
    descSize = 64;

    desc = new float[ descSize ];

    memset( desc, 0, descSize * sizeof( float ) );

    int hw = imgW >> 1;
    int hh = imgH >> 1;

    float upper = logf( sqrtf( 2.0f * hw * hh ) );

    int x, y;
    int accum = 0;
    int offset = 0;
    for ( y = 0; y < imgH; y++ ) {
        for ( x = 0; x < imgW; x++ ) {
            accum += img[ offset ];
        }
    }
    int average = accum / ( imgW * imgH );

    offset = 0;
    for ( y = 0; y < imgH; y++ ) {
        for ( x = 0; x < imgW; x++ ) {
            /*
            int dx =     ( int )img[ offset - imgW - 1 ] +
            		 2 * ( int )img[ offset        - 1 ] +
            			 ( int )img[ offset + imgW - 1 ] -
            			 ( int )img[ offset - imgW + 1 ] -
            		 2 * ( int )img[ offset        + 1 ] -
            		  	 ( int )img[ offset + imgW + 1 ];

            int dy =     ( int )img[ offset - imgW - 1 ] +
            		 2 * ( int )img[ offset - imgW     ] +
            		 	 ( int )img[ offset - imgW + 1 ] -
            			 ( int )img[ offset + imgW - 1 ] -
            		 2 * ( int )img[ offset + imgW     ] -
            		 	 ( int )img[ offset + imgW + 1 ];

            float ori = atan2f( ( float )dy, dx );
            */

            int r = ( x - hw ) * ( x - hw ) + ( y - hh ) * ( y - hh );

            float logr = 0.0f;

            if ( r > 0 ) {
                logr = logf( sqrtf( ( float )r ) );
            }

            float o = atan2f( ( float )y - hh, ( float )x - hw );

            if ( o < -M_PI ) {
                o += TWO_PI;
            } else if ( o > M_PI ) {
                o -= TWO_PI;
            }

            int c = ( int )img[ offset ] - average;

            int bin_r = id_cch_discretize( r, 0, upper, 4 );

            int bin_o = id_cch_discretize( o, -M_PI, M_PI, 8 );

            int bin_c = ( c <= 0 ) ? 0 : 1;

            int bin = bin_o + ( bin_r << 3 ) + ( bin_c << 5 );

            assert( ( bin >= 0 ) && ( bin < 64 ) );

            desc[ bin ] += fabs( c );

            offset++;
        }
    }

    int k;
    float sum = 0;
    for ( k = 0; k < descSize; k++ ) {
        sum += desc[ k ];
    }
    if ( sum > 1e-12 ) {
        for ( k = 0; k < descSize; k++ ) {
            desc[ k ] /= sum;
        }
    }

    return desc;
}


int id_cch_discretize( float v, float lb, float rb, int nbins )
{
    return ( ( int )( ( v - lb ) / ( rb - lb ) * ( nbins + 1 ) ) ) % nbins;
}

//! Load CCH Histograms from a file
void LoadCCHHists(char *filename, vector<CCH> &hists, bool append/*=false*/)
{
//  int histogramLength, count;
  ifstream infile(filename);

  // clear the histograms vector first if required
  if (append == false)
    hists.clear();

  CCH cch = new float[64];
  char buffer[20]={0};
  infile >> buffer;
  while (buffer[0]!=0)
  {
    cch = new float[64];
    cch[0] = atof(buffer);
    // read CCH histograms
    for (int j = 1; j < 64; j++)
        infile >> cch[j];

    hists.push_back(cch);
    buffer[0] = 0;
    infile >> buffer;
  }
//   delete []cch;
}

//! Save the CCH Histogram to a file
void SaveCCHHist(ofstream &outfile, const CCH &hist)
{
  for (int j = 0; j < 64; j++)
    outfile << hist[j] << " ";
  outfile << endl;
}
