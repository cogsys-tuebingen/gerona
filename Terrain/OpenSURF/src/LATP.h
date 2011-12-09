
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

#ifndef _LATP_H
#define _LATP_H

#include <fstream>
#include <vector>
#include <math.h>
#include <cstring>

/* Compare a value pointed to by 'ptr' to the 'center' value and
 * increment pointer. Comparison is made by masking the most
 * significant bit of an integer (the sign) and shifting it to an
 * appropriate position. */
// #define latp_compab_mask_inc(ptr,shift) { value |= ((unsigned int)(*center - *ptr - 1) & 0x80000000) >> (31-shift); ptr++; }
// #define latp_compab_mask_inc(ptr,k,shift) { if((unsigned int)(*center - *ptr + k - 1) < 0) value |= (1 << (shift)) | 0x100; else if((unsigned int)(*center - *ptr - k + 1) > 0) value |= 1 << (shift); ptr++; }
/* Compare a value 'val' to the 'center' value. */
// #define latp_compab_mask(val,shift) { value |= ((unsigned int)(*center - (val) - 1) & 0x80000000) >> (31-shift); }
// #define latp_compab_mask(val,k,shift) { if((unsigned int)(*center - val + k - 1) < 0) value |= (1 << (shift)) | 0x100; else if((unsigned int)(*center - val - k + 1) > 0) value |= 1 << (shift); }
/* Predicate 1 for the 3x3 neighborhood */
#define latp_predicate 1
/* The number of bits */
#define latp_bits 8

typedef struct
{
    int x,y;
} latpIntegerpoint;

typedef struct
{
    double x,y;
} latpDoublepoint;

class LATP
{
public:
    LATP()
    {
        threshold = 5;
        memset(histogramPos, 0, sizeof(int)*256); /* Clear result histogram */
        memset(histogramNeg, 0, sizeof(int)*256); /* Clear result histogram */
    }

    //! Gets the distance in descriptor space between Ipoints
    float operator-(const LATP &rhs)
    {
        float sum=0.f;
        for (int i=0; i < 256; ++i)
            sum += (this->histogramNeg[i] - rhs.histogramNeg[i])*(this->histogramNeg[i] - rhs.histogramNeg[i]);
        for (int i=0; i < 256; ++i)
            sum += (this->histogramPos[i] - rhs.histogramPos[i])*(this->histogramPos[i] - rhs.histogramPos[i]);
        return sqrt(sum);
    };

    //! returns indexed descriptor
    int operator[](int index)
    {
        if (index >= 0 && index < 256)
            return histogramNeg[index];
        else if (index >= 256 && index < 256*2)
            return histogramPos[index-256];
        else
            return 0;
    };

    // threshold (+k, -k)
    float threshold;

    // histogram bins
    int histogramPos[256];
    int histogramNeg[256];
};

void latp_compab_mask_inc(int *&ptr, double ut, double lt, int shift, int &value1, int &value2);
void latp_compab_mask(int val, double ut, double lt, int shift, int &value1, int &value2);
void latp_calculate_points(void);
inline double latp_interpolate_at_ptr(int* upperLeft, int i, int columns);
LATP latp_histogram(int* img, int rows, int columns, float k, LATP &result, int interpolated);
void LoadLATPHists(char *filename, std::vector<LATP> &hists, bool append);
void SaveLATPHist(char *filename, const LATP &hist, bool append);
void SaveLATPHist(std::ofstream &outfile, const LATP &hist);

#endif
