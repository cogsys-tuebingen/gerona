/*
 * CurveRenderer.cpp
 *
 *  Created on: Aug 21, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "CurveRenderer.h"

/// SYSTEM
#include <sstream>
#include <typeinfo>

using namespace lib_path;

CurveRenderer::CurveRenderer(cv::Mat& debug_image)
    : m_debug_image(debug_image)
{
}


void CurveRenderer::snapshot_in_new_window(Curve* curve)
{
    if(m_debug_image.empty()) {
        return;
    }

    draw(curve);

    cv::namedWindow("debug image");
    cv::imshow("debug image", m_debug_image);
    cv::waitKey(0);
}

void CurveRenderer::draw_map(GridMap2d* m_map)
{
    for(unsigned y=0; y<m_map->getHeight(); y++) {
        for(unsigned x=0; x<m_map->getWidth(); x++) {
            cv::Vec3b col;
            if(m_map->isFree(x, y)) {
                col = cv::Vec3b::all(255);
            } else {
                unsigned v = m_map->getValue(x, y);
                col = cv::Vec3b(255-v, 0, 0);
            }

            m_debug_image.at<cv::Vec3b>(m_debug_image.rows-1-y, x) = col;
        }
    }
}

void CurveRenderer::draw(Curve* curve, cv::Scalar bg_color)
{
    if(m_debug_image.empty()) {
        return;
    }

    if(curve->is_valid()) {
        // draw the underlaying geometry
        CurveSegment* s;
        CircleSegment* cs;
        LineSegment* ls;
        std::vector<CurveSegment*>::iterator it;
        for(it = curve->m_sequence.begin(); it != curve->m_sequence.end(); ++it) {
            s = *it;
            if(typeid(*s) == typeid(CircleSegment)) {
                cs = dynamic_cast<CircleSegment*>(s);
                cv::circle(m_debug_image,
                           p2cv(cs->get_center(), m_debug_image.rows), cs->get_radius(),
                           bg_color, 4, CV_AA);
                std::stringstream txt;
                txt << cs->m_angle_step_size;
                cv::Point c(cs->m_center.x, m_debug_image.rows - cs->m_center.y);
                cv::putText(m_debug_image, txt.str(), c, cv::FONT_HERSHEY_SIMPLEX, 0.3,
                            cv::Scalar(0, 0, 0));

            } else if(typeid(*s) == typeid(LineSegment)) {
                ls = dynamic_cast<LineSegment*>(s);
                cv::line(m_debug_image,
                         p2cv(ls->start(), m_debug_image.rows), p2cv(ls->end(), m_debug_image.rows),
                         bg_color, 4, CV_AA);

            } else {
                std::cerr << "CurveRenderer: invalid segmenttype?" << std::endl;
            }
        }

        // draw the trajectory
        Pose2d last = curve->m_start;
        curve->reset_iteration();
        while(curve->has_next()) {
            Pose2d n = curve->next();
            cv::line(m_debug_image, p2cv(last, m_debug_image.rows), p2cv(n, m_debug_image.rows),
                     cv::Scalar(100,100,100,0.1f), 1, CV_AA);
            last = n;
        }

        curve->reset_iteration();
        while(curve->has_next()) {
            Pose2d n = curve->next();
            draw_arrow(curve, n, cvScalar(200,200,200,0), 0.4f);
        }

        draw_arrow(curve, curve->m_goal,cvScalar(50, 50, 200));

        draw_arrow(curve, curve->m_start, cvScalar(50, 200, 50));
    }
}

void CurveRenderer::draw_arrow(Curve* curve, Pose2d& pose, CvScalar color, float scale)
{
    Point2d t(pose);
    Point2d right(12, 5);
    Point2d left(12, -5);
    Point2d dir(18, 0);

    right *= scale;
    left *= scale;
    dir *= scale;

    Point2d tip = t + dir.rotate(pose.theta);
    cv::line(m_debug_image, p2cv(pose, curve->m_map->getHeight()), p2cv(t + dir.rotate(pose.theta), curve->m_map->getHeight()),
           color, 2.0f * scale, CV_AA);
    cv::line(m_debug_image, p2cv(tip, curve->m_map->getHeight()), p2cv(t + left.rotate(pose.theta), curve->m_map->getHeight()),
           color, 2.0f * scale, CV_AA);
    cv::line(m_debug_image, p2cv(tip, curve->m_map->getHeight()), p2cv(t + right.rotate(pose.theta), curve->m_map->getHeight()),
           color, 2.0f * scale, CV_AA);
}

void CurveRenderer::display_overlay(Curve* curve)
{
    int font = CV_FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;

    int x_offset = 20;
    int y_offset = 5;
    int dy = 12;

    std::stringstream ss;

    ss << "circle radius: " << curve->m_circle_radius;
    cv::putText(m_debug_image, ss.str().c_str() , cv::Point(x_offset,y_offset += dy), font, scale, cv::Scalar(255,100,100));

    ss.clear();
    ss.str("");
    ss << "max distance: " << curve->m_max_waypoint_distance;
    cv::putText(m_debug_image, ss.str().c_str() , cv::Point(x_offset,y_offset += dy), font, scale, cv::Scalar(255,100,100));

    ss.clear();
    ss.str("");
    ss << "costs: forward: " << curve->m_cost_forwards;
    cv::putText(m_debug_image, ss.str().c_str() , cv::Point(x_offset,y_offset += dy), font, scale, cv::Scalar(255,100,100));
    ss.clear();
    ss.str("");
    ss << "       backward: " << curve->m_cost_backwards;
    cv::putText(m_debug_image, ss.str().c_str() , cv::Point(x_offset,y_offset += dy), font, scale, cv::Scalar(255,100,100));
    ss.clear();
    ss.str("");
    ss << "       curve: " << curve->m_cost_curve;
    cv::putText(m_debug_image, ss.str().c_str() , cv::Point(x_offset,y_offset += dy), font, scale, cv::Scalar(255,100,100));
    ss.clear();
    ss.str("");
    ss << "       straight: " << curve->m_cost_straight;
    cv::putText(m_debug_image, ss.str().c_str() , cv::Point(x_offset,y_offset += dy), font, scale, cv::Scalar(255,100,100));

    ss.clear();
    ss.str("");
    double w = curve->weight();
    ss << "min weight: ";
    if(w < NOT_FREE) {
        ss << w;
    } else {
        ss << "not free (" << w << ")";
    }

    cv::putText(m_debug_image, ss.str().c_str() , cv::Point(x_offset,y_offset += dy), font, scale, cv::Scalar(255,100,100));

}
