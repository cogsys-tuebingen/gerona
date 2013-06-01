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
#include <utils/LibUtil/CDUniTuebingen.hpp>

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

}

#endif // UTIL_HPP
