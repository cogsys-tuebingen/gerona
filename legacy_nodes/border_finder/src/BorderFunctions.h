#ifndef BORDERFUNCTIONS_H
#define BORDERFUNCTIONS_H

#include "BorderFinder.h"

void SetFloddedCellCounter(int val);
void SetVerbose(int val);

iPoint findLargestFrontier(iPoint* contour, int contourLength, bool* occupied, float* frontierRatio);

void classifyContour(iPoint* contour, int contourLength,
                     iPoint* convexHull, int convexHullLength,
                     bool* occupied, float* border);
int convexHull2D(iPoint* V, int n, iPoint* D, iPoint** H) ;
bool floodfill(uchar* image, int x, int y, iPoint* max, iPoint* min, int width, int height);

// Contour following for region
int contourFollow(uchar* image, iPoint* contour,
                  int left, int top, int right, int bottom,
                  int width, int height, bool* occupied);




#endif // BORDERFUNCTIONS_H
