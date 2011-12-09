
/*********************************************************************
 * Copyright (C) 2010
 * All rights reserved.
 *
 *********************************************************************/

#ifndef _CCH_H
#define _CCH_H

#include <fstream>
#include <vector>
// #include <math.h>

typedef float* CCH;

int id_cch_discretize( float v, float lb, float rb, int nbins );
float* cch_desc(int* img, int imgW, int imgH, float*& desc, int& descSize);
void LoadCCHHists(char *filename, std::vector<CCH> &hists, bool append=false);
void SaveCCHHist(std::ofstream &outfile, const CCH &hists);

#endif
