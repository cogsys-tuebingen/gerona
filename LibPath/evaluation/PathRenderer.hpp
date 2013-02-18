/*
 * PathRenderer.hpp
 *
 *  Created on: Feb 01, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef PATHRENDERER_H
#define PATHRENDERER_H

/// PROJECT
#include "../common/Path.h"
#include "../common/GridMap2d.h"

#include <utils/LibGeneric/Utils.hpp>

/// SYSTEM
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>

namespace lib_path
{

template <int Scale,
         class PointT,
         class PathT,
         class Heuristic,
         template <class, int> class Connector = DirectConnector >
class PathRenderer : public Connector<PointT, Scale>
{
public:
    PathRenderer(const GridMap2d& map, const Pose2d& start, const Pose2d& goal, cv::Mat& out)
        : map_(map), out_(out) {
        start_.x = start.x;
        start_.y = start.y;
        start_.center_x = start.x;
        start_.center_y = start.y;
        start_.theta = start.theta;
        goal_.x = goal.x;
        goal_.y = goal.y;
        goal_.center_x = goal.x;
        goal_.center_y = goal.y;
        goal_.theta = goal.theta;
    }

    void renderMap() {
        assert(out_.rows > 0);
        assert(out_.cols > 0);

        for(unsigned y=0; y<map_.getHeight(); y++) {
            for(unsigned x=0; x<map_.getWidth(); x++) {
                cv::Vec3b col;
                if(map_.isFree(x, y)) {
                    col = cv::Vec3b::all(255);

//                    PointT c;
//                    c.x = x;
//                    c.y = y;
//                    c.center_x = x;
//                    c.center_y = y;
//                    Heuristic::H2T::compute(&c, &goal_);
//                    Heuristic::compute(&c, &goal_);

//                    unsigned v = std::min(255.0, (c.h * 0.5));
                    //unsigned v = std::min(255.0, c.distance);

                    //col = cv::Vec3b(0, 255-v, v);


                } else {
                    col = cv::Vec3b::all(0);
                }

                fill(generic::Int2Type<Scale>(), x, y, col);
            }
        }

        unsigned meterInCells = Scale / map_.getResolution();

        if(meterInCells > 2) {
            for(int y=0; y<out_.rows; y+=meterInCells) {
                for(int x=0; x<out_.cols; x+=meterInCells) {
                    cv::rectangle(out_, cv::Rect(x,y,meterInCells+1,meterInCells+1), cv::Scalar::all(32), 1);
                }
            }
        }
    }

    void fill(generic::Int2Type<1>, unsigned x, unsigned y, cv::Vec3b& col) {
        out_.at<cv::Vec3b>(out_.rows-1-y, x) = col;
    }

    template <int N>
    void fill(generic::Int2Type<N>, unsigned x, unsigned y, cv::Vec3b& col) {
        static const int W = N;

        unsigned sx = N * x;
        unsigned sy = N * y;

        for(unsigned dy = sy; dy < sy+W; ++dy) {
            for(unsigned dx = sx; dx < sx+W; ++dx) {
                out_.at<cv::Vec3b>(out_.rows-1-dy, dx) = col;
            }
        }

        unsigned yy = out_.rows-1-sy-W;
      //  cv::rectangle(out_, cv::Rect(sx, yy, W+1, W+1), cv::Scalar::all(222), 1);
    }

    void render(const PathT& path) {
        if(path.empty()) {
            return;
        }

        const PointT* last = NULL;

        BOOST_FOREACH(const PointT& pt, path) {
            if(last != NULL) {
                Connector<PointT, Scale>::connect(map_, *last, pt, out_);
            }
            last = &pt;
        }
    }

    template <class Pose>
    void draw_arrow(const Pose& pose, CvScalar color = cv::Scalar(128, 128, 255), float scale = 1.0f) {
        Point2d t(pose);
        Point2d right(12, 5);
        Point2d left(12, -5);
        Point2d dir(18, 0);


        unsigned meterInCells = 1 / map_.getResolution();
        double f = (1/18.0) * scale * meterInCells;

        right *= f;
        left *= f;
        dir *= f;

        Point2d tip = t + dir.rotate(pose.theta);
        cv::line(out_, p2cv(pose, map_.getHeight(), Scale), p2cv(t + dir.rotate(pose.theta), map_.getHeight(), Scale),
                 color, 2.0f * scale, CV_AA);
        cv::line(out_, p2cv(tip, map_.getHeight(), Scale), p2cv(t + left.rotate(pose.theta), map_.getHeight(), Scale),
                 color, 2.0f * scale, CV_AA);
        cv::line(out_, p2cv(tip, map_.getHeight(), Scale), p2cv(t + right.rotate(pose.theta), map_.getHeight(), Scale),
                 color, 2.0f * scale, CV_AA);
    }

protected:
    const GridMap2d& map_;
    cv::Mat out_;
    PointT start_;
    PointT goal_;
};

}

#endif // PATHRENDERER_H
