/*********************************************************************
 * Copyright (C) 2002 Maenpaa
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1.Redistributions of source code must retain all copyright
 *   notices, this list of conditions and the following disclaimer.
 *
 * 2.Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 *
 * 3.The name(s) of the author(s) may not be used to endorse or
 *   promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*********************************************************************
 * Contact information
 *
 * Topi Maenpaa
 * Machine Vision Group
 * Department of Electrical and Information Engineering
 * University of Oulu
 * P.O. Box 4500
 * 90014 University of Oulu
 * topiolli@ee.oulu.fi
 * topi.maenpaa@intopii.fi
 *
 * What this file is all about
 * An implementation of the 8-bit LBP operator.
 * This code is a C version of the original C++ code. It is a bit
 * messy because of optimizations and the fact that the code was
 * originally designed for an object-oriented libary.
 * If you find a bug, please inform me.
 ******************************************************************/

#include <iostream>
#include <fstream>
#include <cstring>
#include <math.h>
#include "LATP.h"
#include <stdlib.h>

using namespace std;

latpIntegerpoint latpPoints[latp_bits];
latpDoublepoint latpOffsets[latp_bits];

void latp_compab_mask_inc(int *&ptr, double ut, double lt, int shift, int &value1, int &value2) 
{ 
//   int value = 0;
  if (*ptr >= ut) 
    value1 += pow(2, shift);
  else if (*ptr <= lt)
    value2 += pow(2, shift);
  ptr++; 
}

void latp_compab_mask(int val, double ut, double lt, int shift, int &value1, int &value2) 
{
//   int value = 0;
  if (val >= ut) 
    value1 += pow(2, shift);
  else if (val <= lt) 
    value2 += pow(2, shift);
}

/*
 * Get a bilinearly interpolated value for a pixel.
 */
inline double latp_interpolate_at_ptr(int* upperLeft, int i, int columns)
{
	double dx = 1-latpOffsets[i].x;
	double dy = 1-latpOffsets[i].y;
	return
		*upperLeft*dx*dy +
		*(upperLeft+1)*latpOffsets[i].x*dy +
		*(upperLeft+columns)*dx*latpOffsets[i].y +
		*(upperLeft+columns+1)*latpOffsets[i].x*latpOffsets[i].y;
}
	
/*
 * Calculate the point coordinates for circular sampling of the neighborhood.
 */
void latp_calculate_points(void)
{
	double step = 2 * M_PI / latp_bits, tmpX, tmpY;
	int i;
	for (i=0;i<latp_bits;i++)
  {
    tmpX = latp_predicate * cos(i * step);
    tmpY = latp_predicate * sin(i * step);
    latpPoints[i].x = (int)tmpX;
    latpPoints[i].y = (int)tmpY;
    latpOffsets[i].x = tmpX - latpPoints[i].x;
    latpOffsets[i].y = tmpY - latpPoints[i].y;
    if (latpOffsets[i].x < 1.0e-10 && latpOffsets[i].x > -1.0e-10) /* rounding error */
      latpOffsets[i].x = 0;
    if (latpOffsets[i].y < 1.0e-10 && latpOffsets[i].y > -1.0e-10) /* rounding error */
      latpOffsets[i].y = 0;
    
    if (tmpX < 0 && latpOffsets[i].x != 0)
    {
      latpPoints[i].x -= 1;
      latpOffsets[i].x += 1;
    }
    if (tmpY < 0 && latpOffsets[i].y != 0)
    {
      latpPoints[i].y -= 1;
      latpOffsets[i].y += 1;
    }
  }
}


double Mean(const int arr[], size_t n)
{
  double mean = 0.0;
  for (size_t idx = 0; idx < n; idx++)
  {
    mean += arr[idx];
  }
  mean /= static_cast<double>(n);
  return mean;
}

double StDev(const int arr[], size_t n)
{
  double mean = Mean(arr, n);
  double variance = 0.0;
  for (size_t idx = 0; idx < n; idx++)
  {
    double temp = arr[idx] - mean;
    variance += temp*temp;
  }

  // Compute sample variance using Bessel's correction (see http://en.wikipedia.org/wiki/Bessel%27s_correction)
  variance /= static_cast<double>(n) - (n == 1 ? 0.0 : 1.0);  

  // Standard deviation is square root of variance
  return sqrt(variance);
}

/*
 * Calculate the LATP histogram for an integer-valued image. This is an
 * optimized version of the basic 8-bit LATP operator. Note that this
 * assumes 4-byte integers. In some architectures, one must modify the
 * code to reflect a different integer size.
 * 
 * img: the image data, an array of rows*columns integers arranged in
 * a horizontal raster-scan order
 * rows: the number of rows in the image
 * columns: the number of columns in the image
 * result: an array of 256 integers. Will hold the 256-bin LATP histogram.
 * interpolated: if != 0, a circular sampling of the neighborhood is
 * performed. Each pixel value not matching the discrete image grid
 * exactly is obtained using a bilinear interpolation. You must call
 * calculate_points (only once) prior to using the interpolated version.
 * return value: result
 */
LATP latp_histogram(int* img, int rows, int columns, float k, LATP &result, int interpolated)
{
	int leap = columns*latp_predicate;
	/*Set up a circularly indexed neighborhood using nine pointers.*/
	int
		*p0 = img,
		*p1 = p0 + latp_predicate,
		*p2 = p1 + latp_predicate,
		*p3 = p2 + leap,
		*p4 = p3 + leap,
		*p5 = p4 - latp_predicate,
		*p6 = p5 - latp_predicate,
		*p7 = p6 - leap,
		*center = p7 + latp_predicate;
	int value1, value2;
	int pred2 = latp_predicate << 1;
	int r,c;
  double mean, standard_deviation, upper_threshold=0, lower_threshold=0;
  int neighborhood[9]={0};

 	memset(result.histogramPos,0,sizeof(int)*256); /* Clear result histogram */
  memset(result.histogramNeg,0,sizeof(int)*256); /* Clear result histogram */
		
	if (!interpolated)
  {
    for (r=0;r<rows-pred2;r++)
    {
      for (c=0;c<columns-pred2;c++)
      {
        value1 = value2 = 0;
        neighborhood[0] = *p0;
        neighborhood[1] = *p1;
        neighborhood[2] = *p2;
        neighborhood[3] = *p3;
        neighborhood[4] = *p4;
        neighborhood[5] = *p5;
        neighborhood[6] = *p6;
        neighborhood[7] = *p7;
        neighborhood[8] = *center;
        mean = Mean(neighborhood, 9);
        standard_deviation = StDev(neighborhood, 9);
        upper_threshold = mean + k * standard_deviation;
        lower_threshold = mean - k * standard_deviation;
        /* Unrolled loop */
        latp_compab_mask_inc(p0, upper_threshold, lower_threshold, 0, value1, value2);
        latp_compab_mask_inc(p1, upper_threshold, lower_threshold, 1, value1, value2);
        latp_compab_mask_inc(p2, upper_threshold, lower_threshold, 2, value1, value2);
        latp_compab_mask_inc(p3, upper_threshold, lower_threshold, 3, value1, value2);
        latp_compab_mask_inc(p4, upper_threshold, lower_threshold, 4, value1, value2);
        latp_compab_mask_inc(p5, upper_threshold, lower_threshold, 5, value1, value2);
        latp_compab_mask_inc(p6, upper_threshold, lower_threshold, 6, value1, value2);
        latp_compab_mask_inc(p7, upper_threshold, lower_threshold, 7, value1, value2);
        center++;

//         value %= 512;
//         cout << value << endl;
        result.histogramPos[value1]++; /* Increase histogram bin value */
        result.histogramNeg[value2]++; /* Increase histogram bin value */
      }
      p0 += pred2;
      p1 += pred2;
      p2 += pred2;
      p3 += pred2;
      p4 += pred2;
      p5 += pred2;
      p6 += pred2;
      p7 += pred2;
      center += pred2;
    }
  }
	else
  {
    p0 = center + latpPoints[5].x + latpPoints[5].y * columns;
    p2 = center + latpPoints[7].x + latpPoints[7].y * columns;
    p4 = center + latpPoints[1].x + latpPoints[1].y * columns;
    p6 = center + latpPoints[3].x + latpPoints[3].y * columns;
      
    for (r=0;r<rows-pred2;r++)
    {
      for (c=0;c<columns-pred2;c++)
      {
        value1 = value2 = 0;

        /* Unrolled loop */
        latp_compab_mask_inc(p1,upper_threshold, lower_threshold, 1, value1, value2);
        latp_compab_mask_inc(p3,upper_threshold, lower_threshold, 3, value1, value2);
        latp_compab_mask_inc(p5,upper_threshold, lower_threshold, 5, value1, value2);
        latp_compab_mask_inc(p7,upper_threshold, lower_threshold, 7, value1, value2);

        /* Interpolate corner pixels */
        latp_compab_mask((int)(latp_interpolate_at_ptr(p0,5,columns)+0.5),upper_threshold, lower_threshold, 0, value1, value2);
        latp_compab_mask((int)(latp_interpolate_at_ptr(p2,7,columns)+0.5),upper_threshold, lower_threshold, 2, value1, value2);
        latp_compab_mask((int)(latp_interpolate_at_ptr(p4,1,columns)+0.5),upper_threshold, lower_threshold, 4, value1, value2);
        latp_compab_mask((int)(latp_interpolate_at_ptr(p6,3,columns)+0.5),upper_threshold, lower_threshold, 6, value1, value2);
        p0++;
        p2++;
        p4++;
        p6++;
        center++;
          
//         result[value]++;
        result.histogramPos[value1]++; /* Increase histogram bin value */
        result.histogramNeg[value2]++; /* Increase histogram bin value */
      }
      p0 += pred2;
      p1 += pred2;
      p2 += pred2;
      p3 += pred2;
      p4 += pred2;
      p5 += pred2;
      p6 += pred2;
      p7 += pred2;
      center += pred2;
    }
  }
/*  long sum=0;
  for (int i=0; i<256; i++)
  {
    sum += result.histogramPos[i];
    if (result.histogramPos[i] > 100)
      cout << "Histogram[" << i << "]=" << result.histogramPos[i] << endl;
  }
  for (int i=0; i<256; i++)
  {
    sum += result.histogramNeg[i];
    if (result.histogramNeg[i] > 100)
      cout << "Histogram[" << i << "]=" << result.histogramNeg[i] << endl;
  }
  if (sum != (rows-pred2)*(columns-pred2)*2)
  {
    cout << "Wrong Histogram(sum=" << sum << "<" << (rows-pred2)*(columns-pred2) << ")=";
    for (int i=0; i<256; i++)
      cout << result.histogramPos[i] << " ";
    cout << endl;
  }*/
  return result;
}

//! Load the LATP Histograms from file
void LoadLATPHists(char *filename, std::vector<LATP> &hists, bool append)
{
//   int histogramLength, count;
  std::ifstream infile(filename);

  // clear the histograms vector first
  if (append == false)
    hists.clear();

  // read histogram length/number of histograms
//   infile >> descriptorLength;
//   infile >> count;

  LATP latp;
  // for each histogram
//   for (int i = 0; i < count; i++)
  char buffer[10]={0};
  infile >> buffer;
  while (buffer[0]!=0)
  {
    latp.histogramPos[0] = atoi(buffer);
    // read LBP histograms
    for (int j = 1; j < 256; j++)
      infile >> latp.histogramPos[j];
    for (int j = 0; j < 256; j++)
      infile >> latp.histogramNeg[j];

    hists.push_back(latp);
    buffer[0] = 0;
    infile >> buffer;
  }
}

//! Save the LATP Histogram in file
void SaveLATPHist(char *filename, const LATP &hist, bool append)
{
//   int histogramLength, count;
  std::ofstream outfile;
// 
//   // clear the histograms vector first
  if (append == false)
    outfile.open(filename);
  else
    outfile.open(filename, ios::app);    

    // write LATP histograms
    for (int j = 0; j < 256; j++)
      outfile << hist.histogramPos[j] << " ";
    for (int j = 0; j < 256; j++)
      outfile << hist.histogramNeg[j] << " ";
    outfile << endl;
}

//! Save the LATP Histogram in file
void SaveLATPHist(std::ofstream &outfile, const LATP &hist)
{
    // write LATP histograms
    for (int j = 0; j < 256; j++)
      outfile << hist.histogramPos[j] << " ";
    for (int j = 0; j < 256; j++)
      outfile << hist.histogramNeg[j] << " ";
    outfile << endl;
}
