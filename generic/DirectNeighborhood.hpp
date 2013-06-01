/*
 * DirectNeighborhood.hpp
 *
 *  Created on: Feb 02, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef DIRECT_NEIGHBORHOOD_H
#define DIRECT_NEIGHBORHOOD_H

/// COMPONENT
#include "Common.hpp"
#include "MapManager.hpp"

/// PROJECT
#include <utils/LibUtil/MathHelper.h>

namespace lib_path
{

/**
 * @brief The DirectNeighborhood struct is used to select a holonomic grid neighborhood
 */
template <int N, int distance>
struct DirectNeighborhood {};

template <int N, int distance, class Imp>
struct DirectNeighborhoodBase : public NeighborhoodBase
{
    enum { SIZE = N };
    enum { DISTANCE = distance };

    template <class PointT>
    struct NodeHolder {
        typedef PointT NodeType;
    };

    template <class NodeType>
    static bool isNearEnough(NodeType* goal, NodeType* reference) {
        return std::abs(goal->x - reference->x) <= distance &&
               std::abs(goal->y - reference->y) <= distance;
    }

    template <class T, class Map, class NodeType>
    static void iterateFreeNeighbors(T& algo, Map& map, NodeType* reference) {
        int x = reference->x;
        int y = reference->y;
        for(unsigned i = 0; i < SIZE; ++i) {
            int xx = Imp::dx(x,i);
            int yy = Imp::dy(y,i);
            if(map.contains(xx, yy)) {
                NodeType* n = map.lookup(xx, yy);

                if(!map.isFree(n)) {
                    continue;
                }

                algo.processNeighbor(reference, n, Imp::delta(i));
            }
        }
    }
};


/**
 * @brief The DirectNeighborhoodImp<4, distance> struct implements a 4 neighborhood
 */
template <int distance>
struct DirectNeighborhood<4, distance>
        : public DirectNeighborhoodBase<4, distance, DirectNeighborhood<4, distance> >
{
    typedef DirectNeighborhood<4, distance> Self;
    typedef DirectNeighborhoodBase<4, distance, Self> Parent;
    using Parent::DISTANCE;

    static int dx(int x,int i) {
        switch(i) {
        case 0: return x - DISTANCE;
        case 1: return x;
        case 2: return x + DISTANCE;
        default:
        case 3: return x;
        }
    }
    static int dy(int y,int i) {
        switch(i) {
        case 0: return y;
        case 1: return y - DISTANCE;
        case 2: return y;
        default:
        case 3: return y + DISTANCE;
        }
    }
    static double delta(int i) {
        return DISTANCE;
    }
};


/**
 * @brief The DirectNeighborhood<8, distance> struct implements an 8 neighborhood
 */
template <int distance>
struct DirectNeighborhood<8, distance>
        : public DirectNeighborhoodBase<8, distance, DirectNeighborhood<8, distance> >
{
    typedef DirectNeighborhood<8, distance> Self;
    typedef DirectNeighborhoodBase<8, distance, Self> Parent;
    using Parent::DISTANCE;

    static const double COST_DIAG = M_SQRT2* DISTANCE;
    static const double COST_HORI = DISTANCE;

    static int dx(int x,int i) {
        switch(i) {
        case 0: return x - DISTANCE;
        case 1: return x;
        case 2: return x + DISTANCE;
        case 3: return x - DISTANCE;
        case 4: return x + DISTANCE;
        case 5: return x - DISTANCE;
        case 6: return x;
        default:
        case 7: return x + DISTANCE;
        }
    }
    static int dy(int y,int i) {
        switch(i) {
        case 0: return y-DISTANCE;
        case 1: return y-DISTANCE;
        case 2: return y-DISTANCE;
        case 3: return y;
        case 4: return y;
        case 5: return y+DISTANCE;
        case 6: return y+DISTANCE;
        default:
        case 7: return y+DISTANCE;
        }
    }
    static double delta(int i) {
        switch(i) {
        case 0: return COST_DIAG;
        case 1: return COST_HORI;
        case 2: return COST_DIAG;
        case 3: return COST_HORI;
        case 4: return COST_HORI;
        case 5: return COST_DIAG;
        case 6: return COST_HORI;
        default:
        case 7: return COST_DIAG;
        }
    }
};

}
#endif // DIRECT_NEIGHBORHOOD_H
