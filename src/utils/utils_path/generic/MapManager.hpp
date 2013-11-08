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
#include "../common/Bresenham2d.h"

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace lib_path
{

template <class Param, class Extension>
class Manager :
    public Extension
{
public:
    typedef typename Param::NodeType NodeType;
    typedef typename Param::HeuristicType HeuristicType;

    using Extension::data;
    using Extension::w;
    using Extension::h;

    const GridMap2d* getMap() {
        assert(map_ != NULL);
        return map_;
    }

    void setMap(const GridMap2d* map) {
        map_ = map;
        w = map_->getWidth();
        h = map_->getHeight();

        setMap(generic::Int2Type<HeuristicMapTraits<HeuristicType>::HeuristicUsesMapResolution>(), map);

        initMap();
    }

    void setMap(generic::Int2Type<false>, const GridMap2d* map){}

    void setMap(generic::Int2Type<true>, const GridMap2d* map){
        HeuristicType::setMapResolution(map_->getResolution());
    }

    virtual void initMap() = 0;

    bool isFree(const NodeType* reference) {
        return map_->isFree(reference->x, reference->y);
    }

    bool isFree(const double sx, const double sy, const double ex, const double ey) {
        assert(sx >= 0);
        assert(sy >= 0);
        assert(ex >= 0);
        assert(ey >= 0);

        bresenham.setGrid(map_, std::floor(sx), std::floor(sy), std::floor(ex),std::floor(ey));

        while(bresenham.next()) {
            if(bresenham.isOccupied()) {
                return false;
            }
        }

        return true;
    }

    bool contains(const int x, const int y) {
        return map_->isInMap(x, y);
    }

    bool contains(const double x, const double y) {
        return map_->isInMap((int) std::floor(x), (int) std::floor(y));
    }

protected:
    const GridMap2d* map_;
    Bresenham2d bresenham;
};



template <class Param, class Extension>
struct GridMapManager :
    public Manager<Param, Extension> {
    typedef Manager<Param, Extension> ManagerT;
    typedef typename ManagerT::NodeType NodeType;

    using ManagerT::data;
    using ManagerT::w;
    using ManagerT::h;

    void initMap() {
        data = new NodeType[w * h];
        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                NodeType::init(data[index(x,y)], x, y);
            }
        }
    }

    inline unsigned index(unsigned x, unsigned y, unsigned t = 1) {
        return y * w + x;
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        return &data[index(x,y)];
    }

    NodeType* lookup(const int x, const int y) {
        assert(x >= 0);
        assert(y >= 0);
        return &data[index(x,y)];
    }
    NodeType* lookup(const double x, const double y, double ignored = 0) {
        assert(x >= 0);
        assert(y >= 0);
        return lookup((int) std::floor(x), (int) std::floor(y));
    }
};



template <class Param, class Extension>
class StateSpaceManager :
    public Manager<Param, Extension>
{
public:
    typedef Manager<Param, Extension> ManagerT;
    typedef typename ManagerT::NodeType NodeType;

    using ManagerT::data;
    using ManagerT::w;
    using ManagerT::h;
    using ManagerT::theta_slots;

    void initMap() {
        theta_slots = 12;
        double stheta = 0 - M_PI;
        double dtheta = 2 * M_PI / theta_slots;

        data = new NodeType[w * h * theta_slots];
        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                for(unsigned t = 0; t < theta_slots; ++t) {
                    NodeType::init(data[index(x,y,t)], x, y, stheta + dtheta * t);
                }
            }
        }
    }


    inline unsigned index(unsigned x, unsigned y, unsigned t) {
        return y * w + x + t*w*h;
    }
    inline unsigned angle2index(double theta) {
        return (MathHelper::AngleClamp(theta) + M_PI) / (2 * M_PI) * theta_slots;
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        unsigned t = angle2index(reference.theta);
        return &data[index(x,y,t)];
    }

    NodeType* lookup(const int x, const int y, double theta) {
        assert(x >= 0);
        assert(y >= 0);
        return &data[index(x,y,angle2index(theta))];
    }
    NodeType* lookup(const double x, const double y, double theta) {
        assert(x >= 0);
        assert(y >= 0);
        return lookup((int) std::floor(x), (int) std::floor(y), theta);
    }
};

}

#endif // MAPMANAGER_H
