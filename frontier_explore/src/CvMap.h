#ifndef CVMAP_H
#define CVMAP_H

#include <cv.h>

#define CVMAP_LETHAL_OBSTACLE 20
#define CVMAP_NO_INFORMATION 128
#define CVMAP_OPEN 235

class CvMap {
public:
    double resolution;
    double origin_x;
    double origin_y;
    IplImage* image;
    unsigned char* data;

    CvMap( unsigned int width, unsigned int height, double resolution );
    virtual ~CvMap();

    void resize( const unsigned int width, const unsigned int height );
    double getDistance( const unsigned int idx1, const unsigned int idx2 ) const;
    void toWorld( const unsigned int idx, double &x, double& y ) const;
    void erode( unsigned int iterations = 1 );
    void downsample();

    bool isOpen( const unsigned int idx ) const;
    bool isNoInformation( const unsigned int idx ) const;
    bool isLethalObstacle( const unsigned int idx ) const;
};

#endif // CVMAP_H
