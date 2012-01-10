
// C/C++
#include <cmath>

// Project
#include "CvMap.h"


CvMap::CvMap( unsigned int width, unsigned int height, double res ) :
    resolution( res ),
    origin_x( 0 ),
    origin_y( 0 ),
    image( NULL )
{
    resize( width, height );
}

CvMap::~CvMap() {
    cvReleaseImage( &image );
}

void CvMap::resize(const unsigned int width, const unsigned int height) {
    cvReleaseImage( &image );
    image = cvCreateImage( cvSize(width, height), IPL_DEPTH_8U, 1 );
    data = (unsigned char*)image->imageData;
}

void CvMap::erode(unsigned int iterations) {
    cvErode( image, image, NULL, iterations );
}

void CvMap::downsample() {
    //IplImage* dest = cvCreateImage( cvSize((image->width+1)/2, (image->height+1)/2), IPL_DEPTH_8U, 1 );
    // TODO
}

void CvMap::celltoWorld(const unsigned int cell, double &x, double &y) const {
    x = origin_x + resolution * (double)(cell % image->width) + 0.5 * resolution;
    y = origin_y + resolution * (double)(cell / image->height) + 0.5 * resolution;
}

void CvMap::cellToXY(const unsigned int cell, int &x, int &y) const {
    x = cell % image->width;
    y = cell / image->height;
}

bool CvMap::worldToCell(const double x, const double y, unsigned int &cell) const {
    if ( x >= origin_x + image->width * resolution || y >= origin_y + image->height * resolution )
        return false;

    cell = (unsigned int)((x - origin_x)/resolution);
    cell += image->width * (unsigned int)((y - origin_y)/resolution);
    return true;
}

double CvMap::getDistance(const unsigned int idx1, const unsigned int idx2) const {
    double x1, y1, x2, y2;
    celltoWorld( idx1, x1, y1 );
    celltoWorld( idx2, x2, y2 );
    return std::sqrt( std::pow( x1 - x2, 2 ) + std::pow( y1 - y2, 2 ));
}

bool CvMap::isOpen(const unsigned int idx) const {
    return data[idx] >= CVMAP_OPEN;
}

bool CvMap::isLethalObstacle(const unsigned int idx) const {
    return data[idx] <= CVMAP_LETHAL_OBSTACLE;
}

bool CvMap::isNoInformation(const unsigned int idx) const {
    return data[idx] == CVMAP_NO_INFORMATION;
}
