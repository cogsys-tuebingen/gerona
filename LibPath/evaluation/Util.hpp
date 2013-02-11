/*
 * Util.hpp
 *
 *  Created on: Feb 8, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef UTIL_HPP
#define UTIL_HPP

/// PROJECT
#include "../generic/Common.hpp"
#include "../common/GridMap2d.h"

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace lib_path
{

template <class AnyPoint>
inline cv::Point p2cv(AnyPoint p, int img_height, int scale=1)
{
    return cv::Point(p.x*scale, (img_height-p.y)*scale);
}



template <class T>
inline cv::Point2f p2cvRef(const HybridNode<T>& p, int img_height, int scale=1)
{
    return cv::Point2f(p.center_x*scale, (img_height-p.center_y)*scale);
}

template <class V>
inline cv::Point2f p2cvRef(const V& p, int img_height, int scale=1)
{
    return cv::Point2f(p.x*scale, (img_height-p.y)*scale);
}



template <class PointT, int Scale>
class DirectConnector
{
public:
    void connect(const GridMap2d& map, const PointT& from, const PointT& to, cv::Mat out,
                 CvScalar color = cv::Scalar(128, 128, 255), float scale = 0.5f) {
        connectImp(generic::Int2Type<HasForwardField<PointT>::value>(), map, from, to, out, color, scale);
    }

private:
    void connectImp(generic::Int2Type<1>, const GridMap2d& map, const PointT& from, const PointT& to, cv::Mat out, CvScalar color, float scale) {
        color = from.forward ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);

        connectImp(generic::Int2Type<0>(), map, from, to, out, color, scale);
    }

    void connectImp(generic::Int2Type<0>, const GridMap2d& map, const PointT& from, const PointT& to, cv::Mat out, CvScalar color, float scale) {
        cv::line(out, p2cvRef(from, map.getHeight(), Scale), p2cvRef(to, map.getHeight(), Scale), color, 5.0f * scale, CV_AA);
    }

private:
    template <typename Any>
    class HasForwardField
    {
        typedef char Small;
        class Big
        {
            char dummy[2];
        };

        template <typename Class> static Small test(typeof(&Class::forward)) ;
        template <typename Class> static Big test(...);


    public:
        enum { value = sizeof(test<Any>(0)) == sizeof(Small) };
    };
};

}

#endif // UTIL_HPP
