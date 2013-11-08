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

/**
 * @brief p2cv converts a point to pixel coordinates (assumes .x, .y member fields)
 * @param p the point int world coordinates
 * @param img_height the height of the image
 * @param scale the scale factor
 * @return
 */
template <class AnyPoint>
inline cv::Point p2cv(AnyPoint p, int img_height, int scale=1)
{
    return cv::Point(p.x*scale, (img_height-p.y)*scale);
}



/**
 * @brief p2cvRef converts a point to pixel coordinates (assumes .center_x, .center_y member fields)
 * @param p the point int world coordinates
 * @param img_height the height of the image
 * @param scale the scale factor
 * @return
 */
template <class AnyPoint>
inline cv::Point2f p2cvRef(const HybridNode<AnyPoint>& p, int img_height, int scale=1)
{
    return cv::Point2f(p.center_x*scale, (img_height-p.center_y)*scale);
}


/**
 * @brief p2cvRef converts a point to pixel coordinates (assumes .x, .y member fields)
 * @param p the point int world coordinates
 * @param img_height the height of the image
 * @param scale the scale factor
 * @return
 */
template <class V>
inline cv::Point2f p2cvRef(const V& p, int img_height, int scale=1)
{
    return cv::Point2f(p.x*scale, (img_height-p.y)*scale);
}



template <class PointT, int Scale>
/**
 * @brief The DirectConnector class connects two points with a straight line
 */
class DirectConnector
{
public:
    /**
     * @brief connect draws a straight line between two points
     * @param map Map object used
     * @param from start point
     * @param to end point
     * @param out target image
     * @param color color of the line
     * @param scale a factor multiplied onto the width of the line
     */
    void connect(const GridMap2d& map, const PointT& from, const PointT& to, cv::Mat out,
                 CvScalar color = cv::Scalar(128, 128, 255), float scale = 0.5f) {
        connectImp(generic::Int2Type<HasForwardField<PointT>::value>(), map, from, to, out, color, scale);
    }

private:
    // has forward field = true
    void connectImp(generic::Int2Type<true>, const GridMap2d& map, const PointT& from, const PointT& to, cv::Mat out, CvScalar color, float scale) {
        color = from.forward ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);

        connectImp(generic::Int2Type<false>(), map, from, to, out, color, scale);
    }

    // has forward field = false
    void connectImp(generic::Int2Type<false>, const GridMap2d& map, const PointT& from, const PointT& to, cv::Mat out, CvScalar color, float scale) {
        cv::line(out, p2cvRef(from, map.getHeight(), Scale), p2cvRef(to, map.getHeight(), Scale), color, 5.0f * scale, CV_AA);
    }

private:
    /**
     * @brief The HasForwardField class helps to determine, if a class has a field called 'forward'
     */
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
