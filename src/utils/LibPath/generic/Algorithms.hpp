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
#include "NonHolonomicNeighborhood.hpp"
#include "DirectNeighborhood.hpp"
#include "NodeManager.hpp"
#include "Heuristics.hpp"

#include "../common/GridMap2d.h"
#include "../common/Pose2d.h"

#define DEFINE_CONCRETE_ALGORITHM(name, Heuristic, MapManager, QueueManager) \
    template <class PointT, class MapT, class Neighborhood, int DrawInterval=0, template <class> class Extension = MapManagerExtension, class SubParam = NoSubParameter> \
    class name##Search : public GenericSearchAlgorithm<GenericParameter<PointT,Heuristic,MapT,Neighborhood,QueueManager,DrawInterval, SubParam>,\
                                                       MapManager,Extension> {};\
    template <int DrawInterval, class SubParam, template <class> class Extension, class PointT, class MapT, class Neighborhood>\
    class name##Search_Debug : public name##Search<PointT, MapT, Neighborhood, DrawInterval,Extension, SubParam> {};

namespace lib_path
{
DEFINE_CONCRETE_ALGORITHM(BreadthFirst,
                          NoHeuristic,
                          GridMapManager,
                          QueueManager)

DEFINE_CONCRETE_ALGORITHM(BreadthFirstState,
                          NoHeuristic,
                          StateSpaceManager,
                          QueueManager)

DEFINE_CONCRETE_ALGORITHM(AStar2d,
                          HeuristicL2,
                          GridMapManager,
                          PriorityQueueManager)

DEFINE_CONCRETE_ALGORITHM(AStar2dTaxi,
                          HeuristicL1,
                          GridMapManager,
                          PriorityQueueManager)

DEFINE_CONCRETE_ALGORITHM(AStar2dInf,
                          HeuristicLInf,
                          GridMapManager,
                          PriorityQueueManager)


DEFINE_CONCRETE_ALGORITHM(AStar,
                          HeuristicL2,
                          StateSpaceManager,
                          PriorityQueueManager)
typedef MaxHeuristic<HeuristicL2, HeuristicHolonomicNoObstacles> MH1;

DEFINE_CONCRETE_ALGORITHM(AStarHybridHeuristics,
                          MH1,
                          StateSpaceManager,
                          PriorityQueueManager)
}

#endif // BREADTHFIRSTSEARCH_H
