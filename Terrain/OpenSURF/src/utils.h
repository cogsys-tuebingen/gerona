/*********************************************************** 
*  --- OpenSURF ---                                       *
*  This library is distributed under the GNU GPL. Please   *
*  use the contact form at http://www.chrisevansdev.com    *
*  for more information.                                   *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#ifndef UTILS_H
#define UTILS_H

#include <opencv/cv.h>
#include "ipoint.h"

#include <vector>
#include <fstream>


//! Display error message and terminate program
void error(const char *msg);

//! Show the provided image and wait for keypress
char showImage(const IplImage *img);

//! Show the provided image in titled window and wait for keypress
char showImage(char *title,const IplImage *img);

// Convert image to single channel 32F
IplImage* getGray(const IplImage *img);

//! Draw a single feature on the image
void drawIpoint(IplImage *img, Ipoint &ipt, int tailSize = 0);

//! Draw all the Ipoints in the provided vector
void drawIpoints(IplImage *img, std::vector<Ipoint> &ipts, int tailSize = 0);

//! Draw descriptor windows around Ipoints in the provided vector
void drawWindows(IplImage *img, std::vector<Ipoint> &ipts);

// Draw the FPS figure on the image (requires at least 2 calls)
void drawFPS(IplImage *img);

//! Draw a Point at feature location
void drawPoint(IplImage *img, Ipoint &ipt);

//! Draw a Point at all features
void drawPoints(IplImage *img, std::vector<Ipoint> &ipts);

//! Save the SURF features to file
void saveSingleSurf(std::ofstream &outfile, const Ipoint &ipt, bool attribs, bool save_class=false);

//! Save the SURF features to file
void saveSurf(char *filename, std::vector<Ipoint> &ipts, bool save_class=false);

//! Load the SURF features from file
void loadSurf(char *filename, std::vector<Ipoint> &ipts, bool append=false, bool load_class=false);

//! Round float to nearest integer
inline int fRound(float flt)
{
  return (int) floor(flt+0.5f);
}

#endif
