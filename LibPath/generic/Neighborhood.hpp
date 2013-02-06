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

namespace lib_path
{

class GridNeighbor {};

template <class, int>
struct DirectNeighborhoodImp {
};


template <int distance>
struct DirectNeighborhoodImp<generic::Int2Type<4>, distance> {
    enum { SIZE = 4 };
    enum { DISTANCE = distance };

    static int dx(int x,int i) {
        switch(i){
        case 0: return x-DISTANCE;
        case 1: return x;
        case 2: return x+DISTANCE;
        default: case 3: return x;
        }
    }
    static int dy(int y,int i) {
        switch(i){
        case 0: return y;
        case 1: return y-DISTANCE;
        case 2: return y;
        default: case 3: return y+DISTANCE;
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

    static const double COST_DIAG = M_SQRT2 * DISTANCE;
    static const double COST_HORI = DISTANCE;

    static int dx(int x,int i) {
        switch(i){
        case 0: return x-DISTANCE;
        case 1: return x;
        case 2: return x+DISTANCE;
        case 3: return x-DISTANCE;
        case 4: return x+DISTANCE;
        case 5: return x-DISTANCE;
        case 6: return x;
        default: case 7: return x+DISTANCE;
        }
    }
    static int dy(int y,int i) {
        switch(i){
        case 0: return y-DISTANCE;
        case 1: return y-DISTANCE;
        case 2: return y-DISTANCE;
        case 3: return y;
        case 4: return y;
        case 5: return y+DISTANCE;
        case 6: return y+DISTANCE;
        default: case 7: return y+DISTANCE;
        }
    }
    static double delta(int i) {
        switch(i){
        case 0: return COST_DIAG;
        case 1: return COST_HORI;
        case 2: return COST_DIAG;
        case 3: return COST_HORI;
        case 4: return COST_HORI;
        case 5: return COST_DIAG;
        case 6: return COST_HORI;
        default: case 7: return COST_DIAG;
        }
    }
};

template <int N = 8, int distance = 1>
struct DirectNeighborhood : public DirectNeighborhoodImp<generic::Int2Type<N>, distance> {
};


template <class Neighborhood, class NodeT>
struct NeighborSelection<Neighborhood, GridMapManager, NodeT, GridMap2d, GridNeighbor> :
        public MapManagerSelection<NodeT, GridMap2d, GridMapManager>
{
    typedef MapManagerSelection<NodeT, GridMap2d, GridMapManager> MapManager;

    virtual void forEachFreeNeighbor(NodeT* current, NodeT* neighbor) = 0;

    void iterateFreeNeighbors(NodeT* reference) {
        int x = reference->x;
        int y = reference->y;
        for(unsigned i = 0; i < Neighborhood::SIZE; ++i) {
            int xx = Neighborhood::dx(x,i);
            int yy = Neighborhood::dy(y,i);
            if(MapManager::contains(xx, yy)) {
                NodeT * n = MapManager::lookup(xx, yy);
                n->delta = Neighborhood::delta(i);

                if(!isFree(n)){
                    continue;
                }

                forEachFreeNeighbor(reference, n);
            }
        }
    }

    bool isNearEnough(NodeT* goal, NodeT* reference) {
        return std::abs(goal->x - reference->x) <= Neighborhood::DISTANCE &&
               std::abs(goal->y - reference->y) <= Neighborhood::DISTANCE;
    }
};

}
#endif // NEIGHBORHOOD_H
