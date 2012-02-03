/*
 * CurveRenderer.cpp
 *
 *  Created on: Aug 21, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "CurveRenderer.h"

#include <sstream>
#include <typeinfo>

using namespace lib_path;

CurveRenderer::CurveRenderer(IplImage *debug_image)
  : m_debug_image(debug_image)
{
}


void CurveRenderer::snapshot_in_new_window(Curve * curve)
{
  if(m_debug_image == NULL)
    return;

  draw(curve);

  int max_width = 800;
  int max_height = 600;

  float ratio = m_debug_image->width / (float) m_debug_image->height;
  float des_ratio = max_width / (float) max_height;

  float percent = 100;
  if(ratio > des_ratio){
    percent = max_width / (float) m_debug_image->width * 100.f;
  } else {
    percent = max_height / (float) m_debug_image->height * 100.f;
  }


  IplImage *output = cvCreateImage ( cvSize((int)((m_debug_image->width*percent)/100) ,
                                            (int)((m_debug_image->height*percent)/100) ),
                                     m_debug_image->depth, m_debug_image->nChannels );

  cvResize(m_debug_image, output);

  cvStartWindowThread();

  cvNamedWindow("debug image");

  cvShowImage("debug image", output);

  cvWaitKey(0);

  cvDestroyWindow("debug image");
  cvReleaseImage(&output);
}

void CurveRenderer::draw(Curve * curve){
  if(m_debug_image == NULL)
    return;

  // draw map
  for(unsigned y=0; y<curve->m_map->getHeight(); y++) {
    for(unsigned x=0; x<curve->m_map->getWidth(); x++) {
      CvScalar col;
      if(curve->m_map->isFree(x, y)) {
        col = cvScalarAll(255);
      } else {
        unsigned v = curve->m_map->getValue(x, y);
        col = cvScalar(255-v, 0, 0, 0);
      }
      cvSet2D(m_debug_image, m_debug_image->height-1-y, x, col);
    }
  }


  if(curve->is_valid()){
    // draw the underlaying geometry
    CurveSegment * s;
    CircleSegment * cs;
    LineSegment * ls;
    std::vector<CurveSegment*>::iterator it;
    for(it = curve->m_min_combo.begin(); it != curve->m_min_combo.end(); ++it){
      s = *it;
      if(typeid(*s) == typeid(CircleSegment)){
        cs = dynamic_cast<CircleSegment*> (s);
        cvCircle(m_debug_image,
                 p2cv(cs->center_of_orientation(), m_debug_image->height), cs->radius(),
                 cvScalar(80, 30, 30, 0.4), 4, CV_AA);

      } else if(typeid(*s) == typeid(LineSegment)){
        ls = dynamic_cast<LineSegment*> (s);
        cvLine(m_debug_image,
               p2cv(ls->start(), m_debug_image->height), p2cv(ls->end(), m_debug_image->height),
               cvScalar(80, 30, 30, 0.4), 4, CV_AA);

      } else {
        std::cerr << "CurveRenderer: invalid segmenttype?" << std::endl;
      }
    }

    // draw the trajectory
    Pose2d last = curve->m_start;
    curve->reset_iteration();
    while(curve->has_next()){
      Pose2d n = curve->next();
      cvLine(m_debug_image, p2cv(last, m_debug_image->height), p2cv(n, m_debug_image->height),
             cvScalar(100,100,100,0.1f), 1, CV_AA );
      last = n;
    }

    curve->reset_iteration();
    while(curve->has_next()){
      Pose2d n = curve->next();
      draw_arrow(curve, n, cvScalar(200,200,200,0), 0.4f);
    }

    draw_arrow(curve, curve->m_goal,cvScalar(50, 50, 200));

    draw_arrow(curve, curve->m_start, cvScalar(50, 200, 50));
  }
}

void CurveRenderer::draw_arrow(Curve * curve, Pose2d &pose, CvScalar color, float scale)
{
  Point2d t(pose);
  Point2d right (12, 5);
  Point2d left (12, -5);
  Point2d dir (18, 0);

  right *= scale;
  left *= scale;
  dir *= scale;

  Point2d tip = t + dir.rotate(pose.theta);
  cvLine(m_debug_image, p2cv(pose, curve->m_map->getHeight()), p2cv(t + dir.rotate(pose.theta), curve->m_map->getHeight()),
         color, 2.0f * scale, CV_AA);
  cvLine(m_debug_image, p2cv(tip, curve->m_map->getHeight()), p2cv(t + left.rotate(pose.theta), curve->m_map->getHeight()),
         color, 2.0f * scale, CV_AA);
  cvLine(m_debug_image, p2cv(tip, curve->m_map->getHeight()), p2cv(t + right.rotate(pose.theta), curve->m_map->getHeight()),
         color, 2.0f * scale, CV_AA);
}

void CurveRenderer::display_overlay(Curve * curve)
{
  CvFont font;
  double scale = 0.4;
  int line = 1;
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, scale, scale, 0, line);

  int x_offset = 20;
  int y_offset = 5;
  int dy = 12;

  std::stringstream ss;

  ss << "circle radius: " << curve->m_circle_radius;
  cvPutText(m_debug_image, ss.str().c_str() , cvPoint(x_offset,y_offset += dy), &font, cvScalar(255,100,100));

  ss.clear(); ss.str("");
  ss << "max distance: " << curve->m_max_waypoint_distance;
  cvPutText(m_debug_image, ss.str().c_str() , cvPoint(x_offset,y_offset += dy), &font, cvScalar(255,100,100));

  ss.clear(); ss.str("");
  ss << "costs: forward: " << curve->m_cost_forwards;
  cvPutText(m_debug_image, ss.str().c_str() , cvPoint(x_offset,y_offset += dy), &font, cvScalar(255,100,100));
  ss.clear(); ss.str("");
  ss << "       backward: " << curve->m_cost_backwards;
  cvPutText(m_debug_image, ss.str().c_str() , cvPoint(x_offset,y_offset += dy), &font, cvScalar(255,100,100));
  ss.clear(); ss.str("");
  ss << "       curve: " << curve->m_cost_curve;
  cvPutText(m_debug_image, ss.str().c_str() , cvPoint(x_offset,y_offset += dy), &font, cvScalar(255,100,100));
  ss.clear(); ss.str("");
  ss << "       straight: " << curve->m_cost_straight;
  cvPutText(m_debug_image, ss.str().c_str() , cvPoint(x_offset,y_offset += dy), &font, cvScalar(255,100,100));

  ss.clear(); ss.str("");
  double w = curve->weight();
  ss << "min weight: ";
  if (w < NOT_FREE) {
    ss << w;
  } else {
    ss << "not free (" << w << ")";
  }
  cvPutText(m_debug_image, ss.str().c_str() , cvPoint(x_offset,y_offset += dy), &font, cvScalar(255,100,100));

}
