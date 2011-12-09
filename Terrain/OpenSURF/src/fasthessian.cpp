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

#include "integral.h"
#include "ipoint.h"
#include "utils.h"

#include <vector>

#include "responselayer.h"
#include "fasthessian.h"



using namespace std;

//-------------------------------------------------------

//! Constructor without image
FastHessian::FastHessian(std::vector<Ipoint> &ipts, 
                         const int octaves, const int intervals, const int init_sample, 
                         const float thresh) 
                         : ipts(ipts), i_width(0), i_height(0)
{
  // Save parameter set
  saveParameters(octaves, intervals, init_sample, thresh);
}

//-------------------------------------------------------

//! Constructor with image
FastHessian::FastHessian(IplImage *img, std::vector<Ipoint> &ipts, 
                         const int octaves, const int intervals, const int init_sample, 
                         const float thresh) 
                         : ipts(ipts), i_width(0), i_height(0)
{
  // Save parameter set
  saveParameters(octaves, intervals, init_sample, thresh);

  // Set the current image
  setIntImage(img);
}

//-------------------------------------------------------

FastHessian::~FastHessian()
{
  for (unsigned int i = 0; i < responseMap.size(); ++i)
  {
    delete responseMap[i];
  }
}

//-------------------------------------------------------

//! Save the parameters
void FastHessian::saveParameters(const int octaves, const int intervals, 
                                 const int init_sample, const float thresh)
{
  // Initialise variables with bounds-checked values
  this->octaves = 
    (octaves > 0 && octaves <= 4 ? octaves : OCTAVES);
  this->intervals = 
    (intervals > 0 && intervals <= 4 ? intervals : INTERVALS);
  this->init_sample = 
    (init_sample > 0 && init_sample <= 6 ? init_sample : INIT_SAMPLE);
  this->thresh = (thresh >= 0 ? thresh : THRES);
}


//-------------------------------------------------------

//! Set or re-set the integral image source
void FastHessian::setIntImage(IplImage *img)
{
  // Change the source image
  this->img = img;

  i_height = img->height;
  i_width = img->width;
}

//-------------------------------------------------------

//! Find the image features and write into vector of features
void FastHessian::getIpoints()
{
  // filter index map
  static const int filter_map [OCTAVES][INTERVALS] = {{0,1,2,3}, {1,3,4,5}, {3,5,6,7}, {5,7,8,9}, {7,9,10,11}};

  // Clear the vector of exisiting ipts
  ipts.clear();

  // Build the response map
  buildResponseMap();

  // Get the response layers
  ResponseLayer *b, *m, *t;
  for (int o = 0; o < octaves; ++o) for (int i = 0; i <= 1; ++i)
  {
    b = responseMap.at(filter_map[o][i]);
    m = responseMap.at(filter_map[o][i+1]);
    t = responseMap.at(filter_map[o][i+2]);

    // loop over middle response layer at density of the most 
    // sparse layer (always top), to find maxima across scale and space
    for (int r = 0; r < t->height; ++r)
    {
      for (int c = 0; c < t->width; ++c)
      {
        if (isExtremum(r, c, t, m, b))
        {
          interpolateExtremum(r, c, t, m, b);
        }
      }
    }
  }
}

//-------------------------------------------------------

//! Build map of DoH responses
void FastHessian::buildResponseMap()
{
  // Calculate responses for the first 4 octaves:
  // Oct1: 9,  15, 21, 27
  // Oct2: 15, 27, 39, 51
  // Oct3: 27, 51, 75, 99
  // Oct4: 51, 99, 147,195
  // Oct5: 99, 195,291,387

  // Deallocate memory and clear any existing response layers
  for(unsigned int i = 0; i < responseMap.size(); ++i)  
    delete responseMap[i];
  responseMap.clear();

  // Get image attributes
  int w = (i_width / init_sample);
  int h = (i_height / init_sample);
  int s = (init_sample);

  // Calculate approximated determinant of hessian values
  if (octaves >= 1)
  {
    responseMap.push_back(new ResponseLayer(w,   h,   s,   9));
    responseMap.push_back(new ResponseLayer(w,   h,   s,   15));
    responseMap.push_back(new ResponseLayer(w,   h,   s,   21));
    responseMap.push_back(new ResponseLayer(w,   h,   s,   27));
  }
 
  if (octaves >= 2)
  {
    responseMap.push_back(new ResponseLayer(w/2, h/2, s*2, 39));
    responseMap.push_back(new ResponseLayer(w/2, h/2, s*2, 51));
  }

  if (octaves >= 3)
  {
    responseMap.push_back(new ResponseLayer(w/4, h/4, s*4, 75));
    responseMap.push_back(new ResponseLayer(w/4, h/4, s*4, 99));
  }

  if (octaves >= 4)
  {
    responseMap.push_back(new ResponseLayer(w/8, h/8, s*8, 147));
    responseMap.push_back(new ResponseLayer(w/8, h/8, s*8, 195));
  }

  if (octaves >= 5)
  {
    responseMap.push_back(new ResponseLayer(w/16, h/16, s*16, 291));
    responseMap.push_back(new ResponseLayer(w/16, h/16, s*16, 387));
  }

  // Extract responses from the image
  for (unsigned int i = 0; i < responseMap.size(); ++i)
  {
    buildResponseLayer(responseMap[i]);
  }
}

//-------------------------------------------------------

//! Calculate DoH responses for supplied layer
void FastHessian::buildResponseLayer(ResponseLayer *rl)
{
  float *responses = rl->responses;         // response storage
  unsigned char *laplacian = rl->laplacian; // laplacian sign storage
  int step = rl->step;                      // step size for this filter
  int b = (rl->filter - 1) / 2 + 1;         // border for this filter
  int l = rl->filter / 3;                   // lobe for this filter (filter size / 3)
  int w = rl->filter;                       // filter size
  float inverse_area = 1.f/(w*w);           // normalisation factor
  float Dxx, Dyy, Dxy;

  for(int r, c, ar = 0, index = 0; ar < rl->height; ++ar) 
  {
    for(int ac = 0; ac < rl->width; ++ac, index++) 
    {
      // get the image coordinates
      r = ar * step;
      c = ac * step; 

      // Compute response components
      Dxx = BoxIntegral(img, r - l + 1, c - b, 2*l - 1, w)
          - BoxIntegral(img, r - l + 1, c - l / 2, 2*l - 1, l)*3;
      Dyy = BoxIntegral(img, r - b, c - l + 1, w, 2*l - 1)
          - BoxIntegral(img, r - l / 2, c - l + 1, l, 2*l - 1)*3;
      Dxy = + BoxIntegral(img, r - l, c + 1, l, l)
            + BoxIntegral(img, r + 1, c - l, l, l)
            - BoxIntegral(img, r - l, c - l, l, l)
            - BoxIntegral(img, r + 1, c + 1, l, l);

      // Normalise the filter responses with respect to their size
      Dxx *= inverse_area;
      Dyy *= inverse_area;
      Dxy *= inverse_area;
     
      // Get the determinant of hessian response & laplacian sign
      responses[index] = (Dxx * Dyy - 0.81f * Dxy * Dxy);
      laplacian[index] = (Dxx + Dyy >= 0 ? 1 : 0);

#ifdef RL_DEBUG
      // create list of the image coords for each response
      rl->coords.push_back(std::make_pair<int,int>(r,c));
#endif
    }
  }
}
  
//-------------------------------------------------------

//! Non Maximal Suppression function
int FastHessian::isExtremum(int r, int c, ResponseLayer *t, ResponseLayer *m, ResponseLayer *b)
{
  // bounds check
  int layerBorder = (t->filter + 1) / (2 * t->step);
  if (r <= layerBorder || r >= t->height - layerBorder || c <= layerBorder || c >= t->width - layerBorder)
    return 0;

  // check the candidate point in the middle layer is above thresh 
  float candidate = m->getResponse(r, c, t);
  if (candidate < thresh) 
    return 0; 

  for (int rr = -1; rr <=1; ++rr)
  {
    for (int cc = -1; cc <=1; ++cc)
    {
      // if any response in 3x3x3 is greater candidate not maximum
      if (
        t->getResponse(r+rr, c+cc) >= candidate ||
        ((rr != 0 && cc != 0) && m->getResponse(r+rr, c+cc, t) >= candidate) ||
        b->getResponse(r+rr, c+cc, t) >= candidate
        ) 
        return 0;
    }
  }

  return 1;
}

//-------------------------------------------------------

//! Interpolate scale-space extrema to subpixel accuracy to form an image feature.   
void FastHessian::interpolateExtremum(int r, int c, ResponseLayer *t, ResponseLayer *m, ResponseLayer *b)
{
  // get the step distance between filters
  // check the middle filter is mid way between top and bottom
  int filterStep = (m->filter - b->filter);
  assert(filterStep > 0 && t->filter - m->filter == m->filter - b->filter);
 
  // Get the offsets to the actual location of the extremum
  double xi = 0, xr = 0, xc = 0;
  interpolateStep(r, c, t, m, b, &xi, &xr, &xc );

  // If point is sufficiently close to the actual extremum
  if( fabs( xi ) < 0.5f  &&  fabs( xr ) < 0.5f  &&  fabs( xc ) < 0.5f )
  {
    Ipoint ipt;
    ipt.x = static_cast<float>((c + xc) * t->step);
    ipt.y = static_cast<float>((r + xr) * t->step);
    ipt.scale = static_cast<float>((0.1333f) * (m->filter + xi * filterStep));
    ipt.laplacian = static_cast<int>(m->getLaplacian(r,c,t));
    ipts.push_back(ipt);
  }
}

//-------------------------------------------------------

//! Performs one step of extremum interpolation. 
void FastHessian::interpolateStep(int r, int c, ResponseLayer *t, ResponseLayer *m, ResponseLayer *b, 
                                  double* xi, double* xr, double* xc )
{
  CvMat* dD, * H, * H_inv, X;
  double x[3] = { 0 };

  dD = deriv3D( r, c, t, m, b );
  H = hessian3D( r, c, t, m, b );
  H_inv = cvCreateMat( 3, 3, CV_64FC1 );
  cvInvert( H, H_inv, CV_SVD );
  cvInitMatHeader( &X, 3, 1, CV_64FC1, x, CV_AUTOSTEP );
  cvGEMM( H_inv, dD, -1, NULL, 0, &X, 0 );

  cvReleaseMat( &dD );
  cvReleaseMat( &H );
  cvReleaseMat( &H_inv );

  *xi = x[2];
  *xr = x[1];
  *xc = x[0];
}

//-------------------------------------------------------

//! Computes the partial derivatives in x, y, and scale of a pixel.
CvMat* FastHessian::deriv3D(int r, int c, ResponseLayer *t, ResponseLayer *m, ResponseLayer *b)
{
  CvMat* dI;
  double dx, dy, ds;

  dx = (m->getResponse(r, c + 1, t) - m->getResponse(r, c - 1, t)) / 2.0;
  dy = (m->getResponse(r + 1, c, t) - m->getResponse(r - 1, c, t)) / 2.0;
  ds = (t->getResponse(r, c) - b->getResponse(r, c, t)) / 2.0;
  
  dI = cvCreateMat( 3, 1, CV_64FC1 );
  cvmSet( dI, 0, 0, dx );
  cvmSet( dI, 1, 0, dy );
  cvmSet( dI, 2, 0, ds );

  return dI;
}

//-------------------------------------------------------

//! Computes the 3D Hessian matrix for a pixel.
CvMat* FastHessian::hessian3D(int r, int c, ResponseLayer *t, ResponseLayer *m, ResponseLayer *b)
{
  CvMat* H;
  double v, dxx, dyy, dss, dxy, dxs, dys;

  v = m->getResponse(r, c, t);
  dxx = m->getResponse(r, c + 1, t) + m->getResponse(r, c - 1, t) - 2 * v;
  dyy = m->getResponse(r + 1, c, t) + m->getResponse(r - 1, c, t) - 2 * v;
  dss = t->getResponse(r, c) + b->getResponse(r, c, t) - 2 * v;
  dxy = ( m->getResponse(r + 1, c + 1, t) - m->getResponse(r + 1, c - 1, t) - 
          m->getResponse(r - 1, c + 1, t) + m->getResponse(r - 1, c - 1, t) ) / 4.0;
  dxs = ( t->getResponse(r, c + 1) - t->getResponse(r, c - 1) - 
          b->getResponse(r, c + 1, t) + b->getResponse(r, c - 1, t) ) / 4.0;
  dys = ( t->getResponse(r + 1, c) - t->getResponse(r - 1, c) - 
          b->getResponse(r + 1, c, t) + b->getResponse(r - 1, c, t) ) / 4.0;

  H = cvCreateMat( 3, 3, CV_64FC1 );
  cvmSet( H, 0, 0, dxx );
  cvmSet( H, 0, 1, dxy );
  cvmSet( H, 0, 2, dxs );
  cvmSet( H, 1, 0, dxy );
  cvmSet( H, 1, 1, dyy );
  cvmSet( H, 1, 2, dys );
  cvmSet( H, 2, 0, dxs );
  cvmSet( H, 2, 1, dys );
  cvmSet( H, 2, 2, dss );

  return H;
}

//-------------------------------------------------------