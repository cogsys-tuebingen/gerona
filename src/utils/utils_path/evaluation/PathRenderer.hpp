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

#include <utils_generic/Utils.hpp>
#include "Util.hpp"
#include "../generic/Heuristics.hpp"

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
        : map_(map), out_(out), render_factor(1.0), render_offset(0.0) {
        start_.x = start.x;
        start_.y = start.y;
//        start_.center_x = start.x;
//        start_.center_y = start.y;
        start_.theta = start.theta;
        goal_.x = goal.x;
        goal_.y = goal.y;
//        goal_.center_x = goal.x;
//        goal_.center_y = goal.y;
        goal_.theta = goal.theta;
    }


    void setGoal(const Pose2d& goal) {
        goal_.x = goal.x;
        goal_.y = goal.y;
        goal_.theta = goal.theta;
    }

    void render_factor_mult(double d) {
        render_factor = std::max(0.2, render_factor * d);
        std::cout << "new factor:" << render_factor << std::endl;
    }

    void render_offset_add(double offset) {
        offset_ += offset;
    }

public:
    void renderMap() {
        assert(out_.rows > 0);
        assert(out_.cols > 0);

        for(unsigned y=0; y<map_.getHeight(); y++) {
            for(unsigned x=0; x<map_.getWidth(); x++) {
                cv::Vec3b col;
                if(map_.isFree(x, y)) {
                    col = cv::Vec3b::all(255);

                    PointT c;
                    c.x = x;
                    c.y = y;
                    c.theta = goal_.theta + offset_;
//                    c.center_x = x;
//                    c.center_y = y;
//                    Heuristic::H2T::compute(&c, &goal_);
                    Heuristic::compute(&c, &goal_);

                    unsigned v = std::min(255.0, std::max(0.0, ((ValueReader<PointT>::read(c) + render_offset) * render_factor)));
//                    unsigned v = std::min(255.0, c.distance);

                    col = cv::Vec3b(0, 255-v, v);


                } else {
                    col = cv::Vec3b::all(0);
                }

                fill(generic::Int2Type<Scale>(), x, y, col);
            }
        }

        Pose2d indicator = goal_ + Pose2d(15, 0, 0);
        indicator.theta =  goal_.theta + offset_;

        draw_arrow(indicator, cv::Scalar(128, 0, 0));

        unsigned meterInCells = Scale / map_.getResolution();

        if(meterInCells > 2) {
            for(int y=0; y<out_.rows; y+=meterInCells) {
                for(int x=0; x<out_.cols; x+=meterInCells) {
                    cv::rectangle(out_, cv::Rect(x,y,meterInCells+1,meterInCells+1), cv::Scalar::all(32), 1);
                }
            }
        }
    }

    void fill(generic::Int2Type<true>, unsigned x, unsigned y, cv::Vec3b& col) {
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


        if(N > 2) {
            unsigned yy = out_.rows-1-sy-W;
            cv::rectangle(out_, cv::Rect(sx, yy, W+1, W+1), cv::Scalar::all(127), 1);
        }
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

    void setFocus(int x_img, int y_img) {
        int area = 40;

        int x = x_img /** map_.getResolution()*/ / (double) Scale;
        int y = (out_.rows - y_img) /** map_.getResolution()*/ / (double) Scale;

        int min = std::numeric_limits<int>::max();
        int max = std::numeric_limits<int>::min();

        for(int dy=-area; dy<=area; ++dy) {
            for(int dx=-area; dx<=area; ++dx) {
                int mx = x + dx;
                int my = y + dy;

                cv::Vec3b col;
                if(map_.isInMap(mx, my) && map_.isFree(mx, my)) {
                    PointT c;
                    c.x = mx;
                    c.y = my;
                    c.theta = goal_.theta + offset_;
//                    c.center_x = mx;
//                    c.center_y = my;
//                    Heuristic::H2T::compute(&c, &goal_);
                    Heuristic::compute(&c, &goal_);

                    int v = ValueReader<PointT>::read(c);
//                    unsigned v = std::min(255.0, c.distance);

                    if(v > max) {
                        max = v;
                    }
                    if(v < min) {
                        min = v;
                    }
                }
            }
        }

        render_offset = -min;
        render_factor = 255.0 / (max - min);
    }

private:
    template <class P>
    struct ValueReader {
        template <class V>
        static unsigned read(V& c) {
            return If<NodeTraits<P>::HasHeuristicField, void>::read(c);
        }

    private:
        template <bool, class Dummy> // Dummy is there to make the specialization of <true> not explicit
        struct If {
            template <typename Value>
            static unsigned read(Value& c) {
                return 127;
            }
        };
        template <class Dummy> // Dummy is there to make the specialization of <true> not explicit
        struct If<true, Dummy> {
            template <typename Value>
            static unsigned read(Value& c) {
                return c.h;
            }
        };
    };

protected:
    const GridMap2d& map_;
    cv::Mat out_;
    PointT start_;
    PointT goal_;

    double render_factor;
    double render_offset;
    double offset_;
};

}

#endif // PATHRENDERER_H
