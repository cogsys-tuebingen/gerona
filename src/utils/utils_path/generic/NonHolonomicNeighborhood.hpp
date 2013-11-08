/*
 * NonHolonomicNeighborhood.hpp
 *
 *  Created on: Feb 02, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef NON_HOLONOMIC_NEIGHBORHOOD_H
#define NON_HOLONOMIC_NEIGHBORHOOD_H

/// COMPONENT
#include "Common.hpp"
#include "MapManager.hpp"

/// PROJECT
#include <utils_general/MathHelper.h>

namespace lib_path
{

/**
 * @brief The NonHolonomicNeighborhood struct is used to select a non-holonomic grid neighborhood
 */
template <int distance = 100, int steerangle = 10>
struct NonHolonomicNeighborhood {
    enum { DISTANCE = distance };
    enum { STEER_ANGLE = steerangle };
};



/**
 * @brief The NeighborhoodPolicy<NodeT, NonHolonomicNeighborhood<d, a> > struct specifies how to
 *        iterate nodes with the non-holonomic neighborhood policy
 */
template <class NodeT, int d, int a>
struct NeighborhoodPolicy<NodeT, NonHolonomicNeighborhood<d,a> >
        : public NonHolonomicNeighborhood<d,a> {
    typedef NonHolonomicNeighborhood<d,a> Neighborhood;
    typedef HybridNode<NodeT> NodeType;

    enum { SIZE = 3 };
    enum { DIST = Neighborhood::DISTANCE };
    enum { STEER_ANGLE = Neighborhood::STEER_ANGLE };

    static const double DISTANCE = DIST / 100.0;

    static const double DELTA_THETA = STEER_ANGLE / 10.0 * M_PI / 180.0;

    static double advance(NodeType* reference, int i, double& x_, double& y_, double& theta_, bool& forward_) {
        double t;
        double cost = DISTANCE;

        // choose steering direction
        switch(i) {
        case 0: case 3: // right
            t = reference->theta - DELTA_THETA;
            cost *= 1.1;
            break;
        default:
        case 1: case 4: // straight
            t = reference->theta;
            break;
        case 2: case 5: // left
            t = reference->theta + DELTA_THETA;
            cost *= 1.1;
            break;
        }

        // normalize the angle
        t = MathHelper::AngleClamp(t);

        // check driving direction
        if(i < 3) {
            // forward
            x_ = reference->center_x + std::cos(t) * DISTANCE;
            y_ = reference->center_y + std::sin(t) * DISTANCE;
            forward_ = true;

        } else {
            // backward
            x_ = reference->center_x - std::cos(t) * DISTANCE;
            y_ = reference->center_y - std::sin(t) * DISTANCE;
            forward_ = false;

            // penalize driving backwards
            cost *= 5;
        }

        // penalize directional changes
        if(reference->forward != forward_) {
            cost *= 5;
        }

        theta_ = t;

        return cost;
    }

    template <class T>
    void iterateFreeNeighbors(T& search, NodeType* reference) {
        for(unsigned i = 0; i < SIZE; ++i) {
            double to_x,to_y, to_theta;
            bool forward;
            double cost = advance(reference, i, to_x,to_y,to_theta,forward);

            if(search.contains(to_x, to_y) && search.isFree(reference->center_x,reference->center_y, to_x,to_y)) {
                NodeType* n = search.lookup(to_x, to_y, to_theta);

                if(n == NULL || !search.isFree(n)) {
                    continue;
                }

                if(search.processNeighbor(reference, n, cost))  {
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
#endif // NON_HOLONOMIC_NEIGHBORHOOD_H
