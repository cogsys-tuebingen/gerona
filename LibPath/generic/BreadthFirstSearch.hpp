/*
 * BreadthFirstSearch.hpp
 *
 *  Created on: Feb 03, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef BREADTHFIRSTSEARCH_H
#define BREADTHFIRSTSEARCH_H

/// COMPONENT
#include "SearchAlgorithm.hpp"
#include "Neighborhood.hpp"
#include "NodeManager.hpp"

#include "../common/GridMap2d.h"
#include "../common/Pose2d.h"

#define DEFINE_CONCRETE_ALGORITHM(name, Point, Heuristic, Neighbor, Map, NeighborSelector, MapManager, QueueManager) \
    template <class PointT, class MapT, int Neighborhood=8, int Stepsize=5, int DrawInterval=0> \
    class name##Search : public GenericSearchAlgorithm<Point,Heuristic,Neighbor<Neighborhood, Stepsize>,Map,NeighborSelector,MapManager,QueueManager,DrawInterval> {};\
    template <int DrawInterval, class PointT, class MapT, int Neighborhood=8, int Stepsize=5>\
    class name##Search_Debug : public name##Search<PointT, MapT, Neighborhood, Stepsize, DrawInterval> {};

namespace lib_path
{
DEFINE_CONCRETE_ALGORITHM(BreadthFirst,
                          PointT,
                          NoHeuristic,
                          DirectNeighborhood,
                          MapT,
                          GridNeighbor,
                          GridMapManager,
                          QueueManager)

DEFINE_CONCRETE_ALGORITHM(AStar,
                          PointT,
                          HeuristicDistToGoal,
                          DirectNeighborhood,
                          MapT,
                          GridNeighbor,
                          GridMapManager,
                          PriorityQueueManager)
}

#endif // BREADTHFIRSTSEARCH_H
