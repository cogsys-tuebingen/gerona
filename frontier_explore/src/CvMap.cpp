
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
    IplImage* dest = cvCreateImage( cvSize(image->width/2, image->height/2), IPL_DEPTH_8U, 1 );

    unsigned char* quad[4];
    quad[0] = (unsigned char*)image->imageData;
    quad[1] = (unsigned char*)&image->imageData[1];
    quad[2] = (unsigned char*)&image->imageData[image->widthStep];
    quad[3] = (unsigned char*)&image->imageData[image->widthStep+1];

    unsigned char min;
    for ( int row = 0; row < dest->height; ++row ) {
        for ( int cell = 0; cell < dest->width; ++cell ) {
            min = 255;
            for ( int i = 0; i < 4; ++i ) {
                if ( *quad[i] < min )
                    min = *quad[i];
                quad[i] += 2;
            }
            dest->imageData[row*dest->widthStep + cell] = min;
        }
        quad[0] = (unsigned char*)&image->imageData[2*row*image->widthStep];
        quad[1] = (unsigned char*)&image->imageData[2*row*image->widthStep+1];
        quad[2] = (unsigned char*)&image->imageData[(2*row+1)*image->widthStep];
        quad[3] = (unsigned char*)&image->imageData[(2*row+1)*image->widthStep+1];
    }
    resize( dest->width, dest->height );
    cvCopyImage( dest, image );
    cvReleaseImage( &dest );
    resolution *= 2.0;
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

bool CvMap::worldToXY(const double x, const double y, int &cellX, int &cellY) const {
    unsigned int cell;
    if ( !worldToCell( x, y, cell ))
        return false;
    cellToXY( cell, cellX, cellY );
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
