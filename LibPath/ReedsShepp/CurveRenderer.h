/*
 * CurveRenderer.h
 *
 *  Created on: Aug 21, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef CURVERENDERER_H
#define CURVERENDERER_H

#include "Curve.h"
#include "../common/Point2d.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace lib_path {

inline CvPoint p2cv(Point2d p, int img_height) {
  return cvPoint(p.x, img_height-p.y);
}

class CurveRenderer
{
public:
  /**
   * Constructor, that takes an OpenCV image to draw to
   *
   * @param OpenCV image to draw to
   */
  CurveRenderer(IplImage* debug_image);

  /**
   * Creates a new window and displays a curve
   *
   * @param curve Reeds-Shepp curve to draw in a new window
   */
  void snapshot_in_new_window(Curve * curve);

  /**
   * Draws a curve onto the image of this instance
   *
   * @param curve Reeds-Shepp curve to draw
   */
  void draw(Curve * curve);

  /**
   * Draws an arrow onto the image of this instance
   *
   * @param curve Reeds-Shepp curve (containing the map)
   * @param pose Pose of the arrow
   * @param color Color of the arrow
   * @param scale Scaling factor
   */
  void draw_arrow(Curve * curve, Pose2d &pose, CvScalar color, float scale = 1.0f);

  /**
   * Display additional information
   */
  void display_overlay(Curve * curve);

private:
  IplImage * m_debug_image;
};

}

#endif // CURVERENDERER_H
