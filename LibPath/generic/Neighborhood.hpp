/*
 * Neighborhood.hpp
 *
 *  Created on: Feb 02, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef NEIGHBORHOOD_H
#define NEIGHBORHOOD_H

/// COMPONENT
#include "Common.hpp"
#include "MapManager.hpp"

/// PROJECT
#include <utils/LibUtil/MathHelper.h>

namespace lib_path
{

template <class, int>
struct DirectNeighborhoodImp {
};

template <int distance>
struct DirectNeighborhoodImp<generic::Int2Type<4>, distance> {
    enum { SIZE = 4 };
    enum { DISTANCE = distance };

    static int dx(int x,int i) {
        switch(i) {
        case 0:
            return x-DISTANCE;
        case 1:
            return x;
        case 2:
            return x+DISTANCE;
        default:
        case 3:
            return x;
        }
    }
    static int dy(int y,int i) {
        switch(i) {
        case 0:
            return y;
        case 1:
            return y-DISTANCE;
        case 2:
            return y;
        default:
        case 3:
            return y+DISTANCE;
        }
    }
    static double delta(int i) {
        return DISTANCE;
    }
};

template <int distance>
struct DirectNeighborhoodImp<generic::Int2Type<8>, distance> {
    enum { SIZE = 8 };
    enum { DISTANCE = distance };

    static const double COST_DIAG = M_SQRT2* DISTANCE;
    static const double COST_HORI = DISTANCE;

    static int dx(int x,int i) {
        switch(i) {
        case 0:
            return x-DISTANCE;
        case 1:
            return x;
        case 2:
            return x+DISTANCE;
        case 3:
            return x-DISTANCE;
        case 4:
            return x+DISTANCE;
        case 5:
            return x-DISTANCE;
        case 6:
            return x;
        default:
        case 7:
            return x+DISTANCE;
        }
    }
    static int dy(int y,int i) {
        switch(i) {
        case 0:
            return y-DISTANCE;
        case 1:
            return y-DISTANCE;
        case 2:
            return y-DISTANCE;
        case 3:
            return y;
        case 4:
            return y;
        case 5:
            return y+DISTANCE;
        case 6:
            return y+DISTANCE;
        default:
        case 7:
            return y+DISTANCE;
        }
    }
    static double delta(int i) {
        switch(i) {
        case 0:
            return COST_DIAG;
        case 1:
            return COST_HORI;
        case 2:
            return COST_DIAG;
        case 3:
            return COST_HORI;
        case 4:
            return COST_HORI;
        case 5:
            return COST_DIAG;
        case 6:
            return COST_HORI;
        default:
        case 7:
            return COST_DIAG;
        }
    }
};

template <int n = 8, int distance = 1>
struct DirectNeighborhood {
    enum { N = n };
    enum { DISTANCE = distance };
};

template <class NodeT, int N, int d>
struct NeighborSelection<NodeT, DirectNeighborhood<N,d> > :
    public DirectNeighborhood<N,d>, public DirectNeighborhoodImp<generic::Int2Type<N> ,d> {
    typedef DirectNeighborhoodImp<generic::Int2Type<N> ,d> Neighborhood;
    typedef NodeT NodeType;

    virtual bool forEachFreeNeighbor(NodeType* current, NodeType* neighbor, double delta) = 0;

    template <class Map>
    void iterateFreeNeighbors(Map* map, NodeType* reference) {
        int x = reference->x;
        int y = reference->y;
        for(unsigned i = 0; i < Neighborhood::SIZE; ++i) {
            int xx = Neighborhood::dx(x,i);
            int yy = Neighborhood::dy(y,i);
            if(map->contains(xx, yy)) {
                NodeType* n = map->lookup(xx, yy);

                if(!map->isFree(n)) {
                    continue;
                }

                forEachFreeNeighbor(reference, n, Neighborhood::delta(i));
            }
        }
    }

    bool isNearEnough(NodeType* goal, NodeType* reference) {
        return std::abs(goal->x - reference->x) <= Neighborhood::DISTANCE &&
               std::abs(goal->y - reference->y) <= Neighborhood::DISTANCE;
    }
};





template <int distance = 100, int steerangle = 10>
struct NonHolonomicNeighborhood {
    enum { DISTANCE = distance };
    enum { STEER_ANGLE = steerangle };
};


template <class NodeT, int d, int a>
struct NeighborSelection<NodeT, NonHolonomicNeighborhood<d,a> >
        : public NonHolonomicNeighborhood<d,a> {
    typedef NonHolonomicNeighborhood<d,a> Neighborhood;
    typedef HybridNode<NodeT> NodeType;

    enum { SIZE = 6 };
    enum { DIST = Neighborhood::DISTANCE };
    enum { STEER_ANGLE = Neighborhood::STEER_ANGLE };

    static const double DISTANCE = DIST / 100.0;

    static const double DELTA_THETA = STEER_ANGLE / 10.0 * M_PI / 180.0;

    static double advance(NodeType* reference, int i, double& x_, double& y_, double& theta_, bool& forward_) {
        double t;
        double cost = DISTANCE;

        switch(i) {
        case 0:
            t = reference->theta - DELTA_THETA;
            cost *= 1.2;
            break;
        default:
        case 1:
            t = reference->theta;
            break;
        case 2:
            t = reference->theta + DELTA_THETA;
            cost *= 1.2;
            break;
        }

        t = MathHelper::AngleClamp(t);

        if(i < 3) {
            x_ = reference->center_x + std::cos(t) * DISTANCE;
            y_ = reference->center_y + std::sin(t) * DISTANCE;
            forward_ = true;

        } else {
            x_ = reference->center_x - std::cos(t) * DISTANCE;
            y_ = reference->center_y - std::sin(t) * DISTANCE;
            forward_ = false;
            cost *= 5;
        }

        if(reference->forward != forward_) {
            cost *= 5;
        }

        theta_ = t;

        return cost;
    }

    virtual bool forEachFreeNeighbor(NodeType* current, NodeType* neighbor, double delta) = 0;

    template <class Map>
    void iterateFreeNeighbors(Map* map, NodeType* reference) {
        for(unsigned i = 0; i < SIZE; ++i) {
            double to_x,to_y, to_theta;
            bool forward;
            double cost = advance(reference, i, to_x,to_y,to_theta,forward);

            if(map->contains(to_x, to_y) && map->isFree(reference->center_x,reference->center_y, to_x,to_y)) {
                NodeType* n = map->lookup(to_x, to_y, to_theta);

                if(n == NULL || !map->isFree(n)) {
                    continue;
                }

                if(forEachFreeNeighbor(reference, n, cost))  {
                    n->center_x = to_x;
                    n->center_y = to_y;
                    n->theta = to_theta;
                    n->forward = forward;
                }
            }
        }
    }

    bool isNearEnough(NodeType* goal, NodeType* reference) {
        return std::abs(goal->center_x - reference->center_x) <= DISTANCE / 2 &&
               std::abs(goal->center_y - reference->center_y) <= DISTANCE / 2 &&
               std::abs(MathHelper::AngleClamp(goal->theta - reference->theta)) < M_PI / 16;
    }

};

}
#endif // NEIGHBORHOOD_H
