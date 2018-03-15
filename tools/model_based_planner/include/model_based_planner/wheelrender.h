#ifndef WHEELRENDER_H
#define WHEELRENDER_H

#include <opencv2/core/core.hpp>
#include "wheeldescriptor.h"

/**
 * @brief Helper class for rendering wheel surface
 */
class WheelRender
{
public:
    WheelRender();

    short groundLevel,maxHeight;
    float pixelSize;
    float heightScale;

    WheelDescriptor RenderWheelDesc(float radius, float width, float latRadius,float angle, cv::Point2f jointPos);
    WheelDescriptor RenderWheelDescCyl(float radius, float width,float angle);
    WheelDescriptor RenderWheelDescSph(float radius, float width, float latRadius,float angle);
    void ConvertImg(CVAlignedMat &img, const CVAlignedMat &mask);
    WheelDescriptor CreateDescriptor(cv::Mat wheelImageF, cv::Mat wheelMask, cv::Point2f centerPos, cv::Vec2f dirX, cv::Vec2f dirY);
    double GetEllipsePointZ(double x, double y, double a, double b, double c);

    template <class T>
    double Cross2D(T v1, T v2)
    {
        return v1[0]*v2[1]-v1[1]*v2[0];


    }

    template <class T>
    cv::Mat CropTemplate(cv::Mat in,const cv::Point2f &center, cv::Point2f &newcenter, T testVal = 0 )
    {
        int xmin = in.cols,xmax = 0,ymin = in.rows,ymax = 0;


        for (int ty = 0; ty < in.rows;ty++)
        {
            for (int tx = 0; tx < in.cols;tx++)
            {
                if (in.at<T>(ty,tx) != testVal)
                {
                    if (tx < xmin) xmin = tx;
                    if (tx > xmax) xmax = tx;
                    if (ty < ymin) ymin = ty;
                    if (ty > ymax) ymax = ty;

                }
            }
        }


        xmax++;
        ymax++;


        cv::Rect myROI(xmin, ymin, xmax-xmin, ymax-ymin);

        newcenter.x = center.x - (float)xmin;
        newcenter.y = center.y - (float)ymin;


        // Crop the full image to that image contained by the rectangle myROI
        // Note that this doesn't copy the data
        cv::Mat croppedRef(in, myROI);

        cv::Mat cropped;
        // Copy the data into new matrix
        croppedRef.copyTo(cropped);

        return cropped;

    }

};

#endif // WHEELRENDER_H
