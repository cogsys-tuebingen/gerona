/*
 * MapRenderer.hpp
 *
 *  Created on: Feb 8, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef MAPRENDERER_HPP
#define MAPRENDERER_HPP

/// COMPONENT
#include "Util.hpp"
#include "../generic/Common.hpp"

/// PROJECT
#include <utils/LibGeneric/Utils.hpp>

/// SYSTEM
#include <boost/static_assert.hpp>
#include <opencv2/opencv.hpp>

namespace lib_path
{

template < class Param >
struct MapRenderer : public MapManagerExtension<typename Param::NodeType> {
    enum { Scale = Param::SCALE };

    typedef typename Param::NodeType NodeType;
    typedef MapManagerExtension<NodeType> Ext;

    //BOOST_STATIC_ASSERT(sizeof(NodeType) == sizeof(HybridNode<HeuristicNode<typename Param::PointType> >));

    virtual unsigned index(unsigned x, unsigned y, unsigned t = 0) = 0;

    void visualize(cv::Mat& out) {
        if(Ext::theta_slots == 1) {
            visualize2d(out);
        } else {
            visualize3d(out);
        }
    }

    double findMaxDistance() {
        double maxDistance = 1;
        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                double d = Ext::data[index(x,y)].distance;
                if(d > maxDistance && d != INFINITY) {
                    maxDistance = d;
                }
            }
        }

        return maxDistance;
    }

    void visualize2d(cv::Mat& out) {
        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                for(unsigned t = 0; t < Ext::theta_slots; ++t) {
                    const NodeType& node = Ext::data[index(x,y)];
                    if(node.prev) {
                        NodeType* prev = dynamic_cast<NodeType*>(node.prev);
                        cv::LineIterator it(out, p2cvRef(node, h, Scale), p2cvRef(*prev, h, Scale));
                        for(int i = 0; i < it.count; ++i, ++it) {
                            cv::Vec3b& v = *(cv::Vec3b*) *it;
                            v = cv::Vec3b(v[0], std::max(0, v[1]-50), std::max(0, v[2]-50));
                        }
                    }
                }
            }
        }
    }

    void visualize3d(cv::Mat& out) {
        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                for(unsigned t = 0; t < Ext::theta_slots; ++t) {
                    const NodeType& node = Ext::data[index(x,y,t)];
                    if(node.prev) {
                        NodeType* prev = dynamic_cast<NodeType*>(node.prev);
                        cv::LineIterator it(out, p2cvRef(node, h, Scale), p2cvRef(*prev, h, Scale));
                        for(int i = 0; i < it.count; ++i, ++it) {
                            cv::Vec3b& v = *(cv::Vec3b*) *it;
                            v = cv::Vec3b(std::min(255, v[0]+20), std::min(255, v[1]+20), std::min(255, v[2]+20));
                        }
                    }
                    if(node.isMarked(NodeType::MARK_WATCHED)) {
                        cv::Point p = p2cvRef(node, h, Scale);
                        out.at<cv::Vec3b> (p) = cv::Vec3b(255, 127, 0);
                    }
                }
            }
        }
    }

    inline void fill(generic::Int2Type<1>, cv::Mat& out, unsigned x, unsigned y, uint8_t v) {
        out.at<cv::Vec3b>(out.rows-1-y, x) = cv::Vec3b(255, v, v);
    }

    template <int N>
    inline void fill(generic::Int2Type<N>, cv::Mat& out, unsigned x, unsigned y, uint8_t v) {
        static const int W = N;

        unsigned sx = N * x;
        unsigned sy = N * y;

        for(unsigned dy = sy; dy < sy+W; ++dy) {
            for(unsigned dx = sx; dx < sx+W; ++dx) {
                out.at<cv::Vec3b>(out.rows-1-dy, dx) = cv::Vec3b(255, v, v);
            }
        }
    }

protected:
    unsigned w;
    unsigned h;
};

}

#endif // MAPRENDERER_HPP
