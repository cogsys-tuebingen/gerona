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
template <int n = 8, int distance = 1>
struct DirectNeighborhood {
    enum { N = n };
    enum { DISTANCE = distance };
};

template <int, int>
struct DirectNeighborhoodImp {
    // base case for undefined neighborhood
};


/**
 * @brief The DirectNeighborhoodImp<4, distance> struct implements a 4 neighborhood
 */
template <int distance>
struct DirectNeighborhoodImp<4, distance> {
    enum { SIZE = 4 };
    enum { DISTANCE = distance };

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
 * @brief The DirectNeighborhoodImp<8, distance> struct implements an 8 neighborhood
 */
template <int distance>
struct DirectNeighborhoodImp<8, distance> {
    enum { SIZE = 8 };
    enum { DISTANCE = distance };

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

/**
 * @brief The NeighborhoodPolicy<NodeT, DirectNeighborhood<N, d> > struct specifies how to
 *        iterate nodes with the direct neighborhood policy
 */
template <class NodeT, int N, int d>
struct NeighborhoodPolicy<NodeT, DirectNeighborhood<N,d> > :
    public DirectNeighborhood<N,d>, public DirectNeighborhoodImp<N, d> {
    typedef DirectNeighborhoodImp<N ,d> Neighborhood;
    typedef NodeT NodeType;

    virtual bool processNeighbor(NodeType* current, NodeType* neighbor, double delta) = 0;

    template <class Map>
    void iterateFreeNeighbors(Map& map, NodeType* reference) {
        int x = reference->x;
        int y = reference->y;
        for(unsigned i = 0; i < Neighborhood::SIZE; ++i) {
            int xx = Neighborhood::dx(x,i);
            int yy = Neighborhood::dy(y,i);
            if(map.contains(xx, yy)) {
                NodeType* n = map.lookup(xx, yy);

                if(!map.isFree(n)) {
                    continue;
                }

                processNeighbor(reference, n, Neighborhood::delta(i));
            }
        }
    }

    bool isNearEnough(NodeType* goal, NodeType* reference) {
        return std::abs(goal->x - reference->x) <= Neighborhood::DISTANCE &&
               std::abs(goal->y - reference->y) <= Neighborhood::DISTANCE;
    }
};

}
#endif // DIRECT_NEIGHBORHOOD_H
