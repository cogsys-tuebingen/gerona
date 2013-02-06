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

/// SYSTEM
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>

namespace lib_path
{

template <class AnyPoint>
inline cv::Point p2cv(AnyPoint p, int img_height) {
  return cv::Point(p.x, img_height-p.y);
}
template <class AnyPoint>
inline cv::Point p2cvRef(AnyPoint& p, int img_height) {
  return cv::Point(p.x, img_height-p.y);
}

template <class PointT>
class DirectConnector {
public:
    void connect(const GridMap2d& map, const PointT &from, const PointT &to, cv::Mat out,
                 CvScalar color = cv::Scalar(128, 128, 255), float scale = 0.5f)
    {
        cv::line(out, p2cv(from, map.getHeight()), p2cv(to, map.getHeight()), color, 2.0f * scale, CV_AA);
    }
};

template <class PointT = Pose2d,
          template <class> class Connector = DirectConnector >
class PathRenderer : public Connector<PointT>
{
public:
    PathRenderer(const GridMap2d& map, cv::Mat &out)
            : map_(map), out_(out)
    {
    }

    void renderMap()
    {
        assert(out_.rows > 0);
        assert(out_.cols > 0);
        for(unsigned y=0; y<map_.getHeight(); y++) {
            for(unsigned x=0; x<map_.getWidth(); x++) {
                cv::Vec3b col;
                if(map_.isFree(x, y)) {
                    col = cv::Vec3b::all(255);
                } else {
                    col = cv::Vec3b::all(0);
                }

                out_.at<cv::Vec3b>(out_.rows-1-y, x) = col;
            }
        }
    }

    void renderDistances()
    {
        assert(out_.rows > 0);
        assert(out_.cols > 0);
        for(unsigned y=0; y<map_.getHeight(); y++) {
            for(unsigned x=0; x<map_.getWidth(); x++) {
                cv::Vec3b col;
                if(map_.isFree(x, y)) {
                    col = cv::Vec3b::all(255);
                } else {
                    col = cv::Vec3b::all(0);
                }

                out_.at<cv::Vec3b>(out_.rows-1-y, x) = col;
            }
        }
    }

    void render(const Path& path)
    {
        if(path.empty()){
            return;
        }

        const PointT * last = NULL;

        BOOST_FOREACH(const PointT& pt, path) {
            if(last != NULL){
                Connector<PointT>::connect(map_, *last, pt, out_);
            }
            last = &pt;
        }
    }

    void draw_arrow(const PointT &pose, CvScalar color = cv::Scalar(128, 128, 255), float scale = 1.0f)
    {
        Point2d t(pose);
        Point2d right(12, 5);
        Point2d left(12, -5);
        Point2d dir(18, 0);

        right *= scale;
        left *= scale;
        dir *= scale;

        Point2d tip = t + dir.rotate(pose.theta);
        cv::line(out_, p2cv(pose, map_.getHeight()), p2cv(t + dir.rotate(pose.theta), map_.getHeight()),
               color, 2.0f * scale, CV_AA);
        cv::line(out_, p2cv(tip, map_.getHeight()), p2cv(t + left.rotate(pose.theta), map_.getHeight()),
               color, 2.0f * scale, CV_AA);
        cv::line(out_, p2cv(tip, map_.getHeight()), p2cv(t + right.rotate(pose.theta), map_.getHeight()),
               color, 2.0f * scale, CV_AA);
    }

protected:
    const GridMap2d& map_;
    cv::Mat out_;
};

}

#endif // PATHRENDERER_H
