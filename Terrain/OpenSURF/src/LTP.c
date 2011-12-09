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
#include "LTP.h"
#include <stdlib.h>

using namespace std;

ltpIntegerpoint ltpPoints[ltp_bits];
ltpDoublepoint ltpOffsets[ltp_bits];

void ltp_compab_mask_inc(int *center, int *&ptr, int k, int shift, int &value1, int &value2) 
{ 
//   int value = 0;
  if (*ptr >= *center + k) 
    value1 += pow(2, shift);
  else if (*ptr <= *center - k) 
    value2 += pow(2, shift);
  ptr++; 
}

void ltp_compab_mask(int *center, int val, int k, int shift, int &value1, int &value2) 
{
//   int value = 0;
  if (val >= *center + k) 
    value1 += pow(2, shift);
  else if (val <= *center - k) 
    value2 += pow(2, shift);
}

/*
 * Get a bilinearly interpolated value for a pixel.
 */
inline double ltp_interpolate_at_ptr(int* upperLeft, int i, int columns)
{
	double dx = 1-ltpOffsets[i].x;
	double dy = 1-ltpOffsets[i].y;
	return
		*upperLeft*dx*dy +
		*(upperLeft+1)*ltpOffsets[i].x*dy +
		*(upperLeft+columns)*dx*ltpOffsets[i].y +
		*(upperLeft+columns+1)*ltpOffsets[i].x*ltpOffsets[i].y;
}
	
/*
 * Calculate the point coordinates for circular sampling of the neighborhood.
 */
void ltp_calculate_points(void)
{
	double step = 2 * M_PI / ltp_bits, tmpX, tmpY;
	int i;
	for (i=0;i<ltp_bits;i++)
  {
    tmpX = ltp_predicate * cos(i * step);
    tmpY = ltp_predicate * sin(i * step);
    ltpPoints[i].x = (int)tmpX;
    ltpPoints[i].y = (int)tmpY;
    ltpOffsets[i].x = tmpX - ltpPoints[i].x;
    ltpOffsets[i].y = tmpY - ltpPoints[i].y;
    if (ltpOffsets[i].x < 1.0e-10 && ltpOffsets[i].x > -1.0e-10) /* rounding error */
      ltpOffsets[i].x = 0;
    if (ltpOffsets[i].y < 1.0e-10 && ltpOffsets[i].y > -1.0e-10) /* rounding error */
      ltpOffsets[i].y = 0;
    
    if (tmpX < 0 && ltpOffsets[i].x != 0)
    {
      ltpPoints[i].x -= 1;
      ltpOffsets[i].x += 1;
    }
    if (tmpY < 0 && ltpOffsets[i].y != 0)
    {
      ltpPoints[i].y -= 1;
      ltpOffsets[i].y += 1;
    }
  }
}

/*
 * Calculate the LTP histogram for an integer-valued image. This is an
 * optimized version of the basic 8-bit LTP operator. Note that this
 * assumes 4-byte integers. In some architectures, one must modify the
 * code to reflect a different integer size.
 * 
 * img: the image data, an array of rows*columns integers arranged in
 * a horizontal raster-scan order
 * rows: the number of rows in the image
 * columns: the number of columns in the image
 * result: an array of 256 integers. Will hold the 256-bin LTP histogram.
 * interpolated: if != 0, a circular sampling of the neighborhood is
 * performed. Each pixel value not matching the discrete image grid
 * exactly is obtained using a bilinear interpolation. You must call
 * calculate_points (only once) prior to using the interpolated version.
 * return value: result
 */
LTP ltp_histogram(int* img, int rows, int columns, int k, LTP &result, int interpolated)
{
	int leap = columns*ltp_predicate;
	/*Set up a circularly indexed neighborhood using nine pointers.*/
	int
		*p0 = img,
		*p1 = p0 + ltp_predicate,
		*p2 = p1 + ltp_predicate,
		*p3 = p2 + leap,
		*p4 = p3 + leap,
		*p5 = p4 - ltp_predicate,
		*p6 = p5 - ltp_predicate,
		*p7 = p6 - leap,
		*center = p7 + ltp_predicate;
	int value1, value2;
	int pred2 = ltp_predicate << 1;
	int r,c;

 	memset(result.histogramPos,0,sizeof(int)*256); /* Clear result histogram */
  memset(result.histogramNeg,0,sizeof(int)*256); /* Clear result histogram */
		
	if (!interpolated)
  {
    for (r=0;r<rows-pred2;r++)
    {
      for (c=0;c<columns-pred2;c++)
      {
         value1 = value2 = 0;

        /* Unrolled loop */
        ltp_compab_mask_inc(center,p0,k,0, value1, value2);
        ltp_compab_mask_inc(center,p1,k,1, value1, value2);
        ltp_compab_mask_inc(center,p2,k,2, value1, value2);
        ltp_compab_mask_inc(center,p3,k,3, value1, value2);
        ltp_compab_mask_inc(center,p4,k,4, value1, value2);
        ltp_compab_mask_inc(center,p5,k,5, value1, value2);
        ltp_compab_mask_inc(center,p6,k,6, value1, value2);
        ltp_compab_mask_inc(center,p7,k,7, value1, value2);
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
    p0 = center + ltpPoints[5].x + ltpPoints[5].y * columns;
    p2 = center + ltpPoints[7].x + ltpPoints[7].y * columns;
    p4 = center + ltpPoints[1].x + ltpPoints[1].y * columns;
    p6 = center + ltpPoints[3].x + ltpPoints[3].y * columns;
      
    for (r=0;r<rows-pred2;r++)
    {
      for (c=0;c<columns-pred2;c++)
      {
        value1 = value2 = 0;

        /* Unrolled loop */
        ltp_compab_mask_inc(center,p1,k,1, value1, value2);
        ltp_compab_mask_inc(center,p3,k,3, value1, value2);
        ltp_compab_mask_inc(center,p5,k,5, value1, value2);
        ltp_compab_mask_inc(center,p7,k,7, value1, value2);

        /* Interpolate corner pixels */
        ltp_compab_mask(center,(int)(ltp_interpolate_at_ptr(p0,5,columns)+0.5),k,0, value1, value2);
        ltp_compab_mask(center,(int)(ltp_interpolate_at_ptr(p2,7,columns)+0.5),k,2, value1, value2);
        ltp_compab_mask(center,(int)(ltp_interpolate_at_ptr(p4,1,columns)+0.5),k,4, value1, value2);
        ltp_compab_mask(center,(int)(ltp_interpolate_at_ptr(p6,3,columns)+0.5),k,6, value1, value2);
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

//! Load the LTP Histograms from file
void LoadLTPHists(char *filename, std::vector<LTP> &hists, bool append, bool classes)
{
//   int histogramLength, count;
  std::ifstream infile(filename);

  // clear the histograms vector first
  if (append == false)
    hists.clear();

  // read histogram length/number of histograms
//   infile >> descriptorLength;
//   infile >> count;

  LTP ltp;
  // for each histogram
//   for (int i = 0; i < count; i++)
  char buffer[10]={0};
  infile >> buffer;
  while (buffer[0]!=0)
  {
    ltp.histogramPos[0] = atoi(buffer);
    // read LBP histograms
    for (int j = 1; j < 256; j++)
      infile >> ltp.histogramPos[j];
    for (int j = 0; j < 256; j++)
      infile >> ltp.histogramNeg[j];

    if (classes)
      infile >> ltp.class_name;
    
    hists.push_back(ltp);
    buffer[0] = 0;
    infile >> buffer;
  }
}

//! Save the LTP Histogram in file
void SaveLTPHist(char *filename, const LTP &hist, bool append, char *class_name)
{
//   int histogramLength, count;
  std::ofstream outfile;
// 
//   // clear the histograms vector first
  if (append == false)
    outfile.open(filename);
  else
    outfile.open(filename, ios::app);    

  // write LTP histograms
  for (int j = 0; j < 256; j++)
    outfile << hist.histogramPos[j] << " ";
  for (int j = 0; j < 256; j++)
    outfile << hist.histogramNeg[j] << " ";
  
  if (class_name!=NULL)
    if (class_name[0]!=0)
      outfile << class_name;

  outfile << endl;
  outfile.close();
}

//! Save the LTP Histogram in file
void SaveLTPHist(std::ofstream &outfile, const LTP &hist)
{
    // write LTP histograms
    for (int j = 0; j < 256; j++)
      outfile << hist.histogramPos[j] << " ";
    for (int j = 0; j < 256; j++)
      outfile << hist.histogramNeg[j] << " ";
    outfile << endl;
}
