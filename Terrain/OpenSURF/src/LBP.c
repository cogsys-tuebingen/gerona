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

#include <fstream>
#include <cstring>
#include <math.h>
#include "LBP.h"
#include <stdlib.h>

using namespace std;

integerpoint points[bits];
doublepoint offsets[bits];

/*
 * Get a bilinearly interpolated value for a pixel.
 */
inline double interpolate_at_ptr(int* upperLeft, int i, int columns)
{
	double dx = 1-offsets[i].x;
	double dy = 1-offsets[i].y;
	return
		*upperLeft*dx*dy +
		*(upperLeft+1)*offsets[i].x*dy +
		*(upperLeft+columns)*dx*offsets[i].y +
		*(upperLeft+columns+1)*offsets[i].x*offsets[i].y;
}
	
/*
 * Calculate the point coordinates for circular sampling of the neighborhood.
 */
void calculate_points(void)
{
	double step = 2 * M_PI / bits, tmpX, tmpY;
	int i;
	for (i=0;i<bits;i++)
  {
    tmpX = predicate * cos(i * step);
    tmpY = predicate * sin(i * step);
    points[i].x = (int)tmpX;
    points[i].y = (int)tmpY;
    offsets[i].x = tmpX - points[i].x;
    offsets[i].y = tmpY - points[i].y;
    if (offsets[i].x < 1.0e-10 && offsets[i].x > -1.0e-10) /* rounding error */
      offsets[i].x = 0;
    if (offsets[i].y < 1.0e-10 && offsets[i].y > -1.0e-10) /* rounding error */
      offsets[i].y = 0;
    
    if (tmpX < 0 && offsets[i].x != 0)
    {
      points[i].x -= 1;
      offsets[i].x += 1;
    }
    if (tmpY < 0 && offsets[i].y != 0)
    {
      points[i].y -= 1;
      offsets[i].y += 1;
    }
  }
}

/*
 * Calculate the LBP histogram for an integer-valued image. This is an
 * optimized version of the basic 8-bit LBP operator. Note that this
 * assumes 4-byte integers. In some architectures, one must modify the
 * code to reflect a different integer size.
 * 
 * img: the image data, an array of rows*columns integers arranged in
 * a horizontal raster-scan order
 * rows: the number of rows in the image
 * columns: the number of columns in the image
 * result: an array of 256 integers. Will hold the 256-bin LBP histogram.
 * interpolated: if != 0, a circular sampling of the neighborhood is
 * performed. Each pixel value not matching the discrete image grid
 * exactly is obtained using a bilinear interpolation. You must call
 * calculate_points (only once) prior to using the interpolated version.
 * return value: result
 */
int* lbp_histogram(int* img, int rows, int columns, int* result, int interpolated)
{
	int leap = columns*predicate;
	/*Set up a circularly indexed neighborhood using nine pointers.*/
	int
		*p0 = img,
		*p1 = p0 + predicate,
		*p2 = p1 + predicate,
		*p3 = p2 + leap,
		*p4 = p3 + leap,
		*p5 = p4 - predicate,
		*p6 = p5 - predicate,
		*p7 = p6 - leap,
		*center = p7 + predicate;
	unsigned int value;
	int pred2 = predicate << 1;
	int r,c;

	memset(result,0,sizeof(int)*256); /* Clear result histogram */
		
	if (!interpolated)
  {
    for (r=0;r<rows-pred2;r++)
    {
      for (c=0;c<columns-pred2;c++)
      {
        value = 0;

        /* Unrolled loop */
        compab_mask_inc(p0,0);
        compab_mask_inc(p1,1);
        compab_mask_inc(p2,2);
        compab_mask_inc(p3,3);
        compab_mask_inc(p4,4);
        compab_mask_inc(p5,5);
        compab_mask_inc(p6,6);
        compab_mask_inc(p7,7);
        center++;

        result[value]++; /* Increase histogram bin value */
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
    p0 = center + points[5].x + points[5].y * columns;
    p2 = center + points[7].x + points[7].y * columns;
    p4 = center + points[1].x + points[1].y * columns;
    p6 = center + points[3].x + points[3].y * columns;
      
    for (r=0;r<rows-pred2;r++)
    {
      for (c=0;c<columns-pred2;c++)
      {
        value = 0;

        /* Unrolled loop */
        compab_mask_inc(p1,1);
        compab_mask_inc(p3,3);
        compab_mask_inc(p5,5);
        compab_mask_inc(p7,7);

        /* Interpolate corner pixels */
        compab_mask((int)(interpolate_at_ptr(p0,5,columns)+0.5),0);
        compab_mask((int)(interpolate_at_ptr(p2,7,columns)+0.5),2);
        compab_mask((int)(interpolate_at_ptr(p4,1,columns)+0.5),4);
        compab_mask((int)(interpolate_at_ptr(p6,3,columns)+0.5),6);
        p0++;
        p2++;
        p4++;
        p6++;
        center++;
          
        result[value]++;
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
  return result;
}

//! Load the LBP Histograms from file
void LoadLBPHists(char *filename, std::vector<LBP> &hists, bool append, bool classes)
{
//   int histogramLength, count;
  std::ifstream infile(filename);

  // clear the histograms vector first
  if (append == false)
    hists.clear();

  // read histogram length/number of histograms
//   infile >> descriptorLength;
//   infile >> count;

  LBP lbp;
  // for each histogram
//   for (int i = 0; i < count; i++)
  char buffer[10]={0};
  infile >> buffer;
  while (buffer[0]!=0)
  {
    lbp.histogram[0] = atoi(buffer);
    // read LBP histograms
    for (int j = 1; j < 256; j++)
      infile >> lbp.histogram[j];
    
    if (classes)
      infile >> lbp.class_name;

    hists.push_back(lbp);
    buffer[0] = 0;
    infile >> buffer;
  }
}

//! Save the LBP Histogram to a file
void SaveLBPHist(ofstream &outfile, const LBP &hist, char *class_name)
{
//   int histogramLength, count;
//   std::ofstream outfile;
// 
//   // clear the histograms vector first
//   if (append == false)
//     outfile.open(filename);
//   else
//     outfile.open(filename, ios::app);    

    // write LBP histogram
    for (int j = 0; j < 256; j++)
      outfile << hist.histogram[j] << " ";
    
    if (class_name)
      outfile << hist.class_name;
    
    outfile << endl;
}
