/*
 * CurveRenderer.h
 *
 *  Created on: Aug 21, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef CURVERENDERER_H
#define CURVERENDERER_H

/// COMPONENT
#include "Curve.h"

/// PROJECT
#include "../common/Point2d.h"

/// SYSTE;
#include <opencv2/opencv.hpp>

namespace lib_path
{

inline cv::Point p2cv(Point2d p, int img_height)
{
    return cv::Point(p.x, img_height-p.y);
}

/**
 * The CurveRenderer class is a helper class to visualize a curve for debugging.
 */
class CurveRenderer
{
public:
    /**
     * Constructor, that takes an OpenCV image to draw to
     *
     * @param OpenCV image to draw to
     */
    CurveRenderer(cv::Mat& debug_image);

    /**
     * Creates a new window and displays a curve
     *
     * @param curve Reeds-Shepp curve to draw in a new window
     */
    void snapshot_in_new_window(Curve* curve);

    /**
     * Draws a map onto the image of this instance
     *
     * @param m_map the map to render
     */
    void draw_map(GridMap2d* m_map);

    /**
     * Draws a curve onto the image of this instance
     *
     * @param curve Reeds-Shepp curve to draw
     */
    void draw(Curve* curve, cv::Scalar bg_color = cv::Scalar(80, 30, 30, 0.4));

    /**
     * Draws an arrow onto the image of this instance
     *
     * @param curve Reeds-Shepp curve (containing the map)
     * @param pose Pose of the arrow
     * @param color Color of the arrow
     * @param scale Scaling factor
     */
    void draw_arrow(Curve* curve, Pose2d& pose, CvScalar color, float scale = 1.0f);

    /**
     * Display additional information
     */
    void display_overlay(Curve* curve);

private:
    cv::Mat m_debug_image;
};

}

#endif // CURVERENDERER_H
