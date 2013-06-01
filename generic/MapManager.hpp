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
#include "Heuristics.hpp"

/// PROJECT
#include "../common/GridMap2d.h"
#include "../common/Bresenham2d.h"

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace lib_path
{

template <class NodeT, class MapType>
class Manager
{
public:
    typedef MapType MapT;
    typedef NodeT NodeType;

    template <class PointT>
    struct NodeHolder {
        typedef PointT NodeType;
    };

    Manager()
        : map_(NULL), data(NULL), w(0), h(0), theta_slots(1)
    {}

    virtual ~Manager() {
        if(data != NULL) {
            delete [] data;
        }
    }

    const MapT* getMap() {
        assert(map_ != NULL);
        return map_;
    }

    void setMap(const MapT* map) {
        map_ = map;

        bool replace = w != map->getWidth() || h != map_->getHeight();

        w = map->getWidth();
        h = map->getHeight();

        initMap(replace);
    }

    double getResolution() {
        return map_->getResolution();
    }

    virtual void initMap(bool replace) = 0;

    bool isFree(const NodeType* reference) {
        return map_->isFree(reference->x, reference->y);
    }
    bool isFree(int x, int y) {
        return map_->isFree(x, y);
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

public:
    unsigned w;
    unsigned h;
    unsigned theta_slots;

protected:
    const MapT* map_;
    Bresenham2d bresenham;

    NodeType* data;

};



template <class NodeT>
struct GridMapManager :
    public Manager<NodeT, GridMap2d>
{
    typedef Manager<NodeT, GridMap2d> ManagerT;
    typedef typename ManagerT::NodeType NodeType;

    template <class PointT>
    struct NodeHolder {
        typedef PointT NodeType;
    };

    using ManagerT::data;
    using ManagerT::w;
    using ManagerT::h;

    void initMap(bool replace) {
        bool alloc = data == NULL || replace;
        bool delete_first = alloc && data == NULL;

        if(delete_first) {
            delete[] data;
        }

        if(alloc) {
            std::cout << "allocate new map" << std::endl;
            data = new NodeType[w * h];
        }
        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                NodeType::init(data[index(x,y)], x, y);
            }
        }
    }

    inline unsigned index(unsigned x, unsigned y, unsigned t = 1, bool f = true) {
        return y * w + x;
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        assert(x >= 0);
        assert(y >= 0);
        assert(x < w);
        assert(y < h);
        return &data[index(x,y)];
    }

    NodeType* lookup(const int x, const int y) {
        assert(x >= 0);
        assert(y >= 0);
        assert(x < w);
        assert(y < h);
        return &data[index(x,y)];
    }
    NodeType* lookup(const double x, const double y, double ignored = 0) {
        assert(x >= 0);
        assert(y >= 0);
        return lookup((int) std::floor(x), (int) std::floor(y));
    }
};



template <class NodeT>
class StateSpaceManager :
    public Manager<NodeT, GridMap2d>
{
public:
    typedef Manager<NodeT, GridMap2d> ManagerT;
    typedef typename ManagerT::NodeType NodeType;

    template <class PointT>
    struct NodeHolder {
        typedef PointT NodeType;
    };

    using ManagerT::data;
    using ManagerT::w;
    using ManagerT::h;
    using ManagerT::theta_slots;

    void initMap(bool replace) {
        theta_slots = 36;
        double stheta = 0 - M_PI;
        double dtheta = 2 * M_PI / theta_slots;

        bool alloc = data == NULL || replace;
        bool delete_first = alloc && data == NULL;

        if(delete_first) {
            delete[] data;
        }

        if(alloc) {
            std::cout << "allocate new map" << std::endl;
            data = new NodeType[w * h * theta_slots];
        }

        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                for(unsigned t = 0; t < theta_slots; ++t) {
                    NodeType::init(data[index(x,y,t)], x, y, stheta + dtheta * t);
                }
            }
        }
    }


    inline unsigned index(unsigned x, unsigned y, unsigned t) {
        assert(x >= 0);
        assert(y >= 0);
        assert(x < w);
        assert(y < h);
        assert(t >= 0);
        assert(t < theta_slots);
        return y * w + x + t*w*h;
    }
    inline unsigned angle2index(double theta) {
        return (MathHelper::AngleClamp(theta) + M_PI) / (2 * M_PI) * (theta_slots - 1);
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
        assert(x < w);
        assert(y < h);
        return &data[index(x,y,angle2index(theta))];
    }
    NodeType* lookup(const double x, const double y, double theta) {
        assert(x >= 0);
        assert(y >= 0);
        assert(x < w);
        assert(y < h);
        return lookup((int) std::floor(x), (int) std::floor(y), theta);
    }
};



template <class NodeT>
class DirectionalStateSpaceManager :
    public Manager<NodeT, GridMap2d>
{
public:
    typedef Manager<NodeT, GridMap2d> ManagerT;
    typedef typename ManagerT::NodeType NodeType;

    using ManagerT::data;
    using ManagerT::w;
    using ManagerT::h;
    using ManagerT::theta_slots;

    template <class PointT>
    struct NodeHolder {
        typedef DirectionalNode<PointT> NodeType;
    };

    void initMap(bool replace) {

        theta_slots = 32;
        double stheta = 0 - M_PI;
        double dtheta = 2 * M_PI / theta_slots;

        dimension = w * h * theta_slots;

        bool alloc = data == NULL || replace;
        bool delete_first = alloc && data == NULL;

        if(delete_first) {
            delete[] data;
        }

        if(alloc) {
            std::cout << "allocate new map" << std::endl;
            data = new NodeType[2 * dimension];
        }


        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                for(unsigned t = 0; t < theta_slots; ++t) {
                    for(unsigned f = 0; f <= 1; ++f) {
                        NodeType::init(data[index(x,y,t, f)], x, y, stheta + dtheta * t);
                    }
                }
            }
        }
    }


    inline unsigned index(unsigned x, unsigned y, unsigned t=0, bool forward=true) {
        assert(x >= 0);
        assert(y >= 0);
        assert(x < w);
        assert(y < h);
        assert(t >= 0);
        assert(t < theta_slots);
        return y * w + x + t*w*h + (forward ? 0 : dimension);
    }
    inline unsigned angle2index(double theta) {
        return (MathHelper::AngleClamp(theta) + M_PI) / (2 * M_PI) * (theta_slots - 1);
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        unsigned t = angle2index(reference.theta);
        return &data[index(x,y,t, reference.forward)];
    }
    NodeType* lookup(const Pose2d& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        unsigned t = angle2index(reference.theta);
        return &data[index(x,y,t, true)];
    }

    NodeType* lookup(const int x, const int y, double theta, bool forward) {
        assert(x >= 0);
        assert(y >= 0);
        assert(x < w);
        assert(y < h);
        return &data[index(x,y,angle2index(theta), forward)];
    }
    NodeType* lookup(const double x, const double y, double theta, bool forward) {
        assert(x >= 0);
        assert(y >= 0);
        assert(x < w);
        assert(y < h);
        return lookup((int) std::floor(x), (int) std::floor(y), theta, forward);
    }

private:
    unsigned dimension;
};

}

#endif // MAPMANAGER_H
