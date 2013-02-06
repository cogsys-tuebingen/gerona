/*
 * MapManager.hpp
 *
 *  Created on: Feb 02, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef MAPMANAGER_H
#define MAPMANAGER_H

/// COMPONENT
#include "Common.hpp"

/// PROJECT
#include "../common/GridMap2d.h"

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace lib_path
{

class GridMapManager {};

template <class NodeT>
class MapManagerSelection<NodeT, GridMap2d, GridMapManager> {
public:
    MapManagerSelection()
        : w(0), h(0)
    {
    }

    void setMap(const GridMap2d* map) {
        map_ = map;
        w = map_->getWidth();
        h = map_->getHeight();

        data = new NodeT[w * h];
        for(unsigned y = 0; y < h; ++y){
            for(unsigned x = 0; x < w; ++x){
                data[y * w + x].x = x;
                data[y * w + x].y = y;
                data[y * w + x].distance = INFINITY;
            }
        }
    }

    void visualize(cv::Mat& out) {
        double maxDistance = 1;
        for(unsigned y = 0; y < h; ++y){
            for(unsigned x = 0; x < w; ++x){
                double d = data[y * w + x].distance;
                if(d > maxDistance && d != INFINITY){
                    maxDistance = d;
                }
            }
        }

        for(unsigned y = 0; y < h; ++y){
            for(unsigned x = 0; x < w; ++x){
                const NodeT &node = data[y * w + x];
                double d = node.distance;

                if(d == INFINITY){
                    continue;
                }

                uint8_t v = d / maxDistance * 255;
                out.at<cv::Vec3b>(out.rows-1-y, x) = cv::Vec3b(255, v, v);

                if(node.prev){
                    cv::line(out, p2cvRef(node, h), p2cvRef(*node.prev, h), cv::Scalar(255,200,200));
                }
            }
        }
    }

    virtual ~MapManagerSelection() {
        delete [] data;
    }

    bool isFree(const NodeT* reference) {
        return map_->isFree(reference->x, reference->y);
    }

    template <class PointT>
    NodeT* lookup(const PointT& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        return &data[y * w + x];
    }

    NodeT* lookup(const unsigned x, const unsigned y) {
        return &data[y * w + x];
    }

    bool contains(const int x, const int y) {
        return map_->isInMap(x, y);
    }

protected:
    const GridMap2d* map_;
    unsigned w;
    unsigned h;

private:
    NodeT* data;
};

}

#endif // MAPMANAGER_H
