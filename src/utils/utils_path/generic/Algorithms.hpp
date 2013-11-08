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

#define DEFINE_ALGORITHM(name, Heuristic, MapManager, QueueManager) \
    template <class Neighborhood, class AnalyticExpansion = NoExpansion, class PointT = Pose2d, class MapT = GridMap2d, int DrawInterval=0, int DrawStartInterval=0> \
    class name##Search : public GenericSearchAlgorithm<GenericParameter<PointT,Heuristic,MapT,Neighborhood,AnalyticExpansion,QueueManager,MapManager,DrawInterval, DrawStartInterval> > {};\
    template <int DrawInterval, int DrawStartInterval, class Neighborhood, class AnalyticExpansion = NoExpansion, class PointT = Pose2d, class MapT = GridMap2d>\
    class name##Search_Debug : public name##Search<Neighborhood, AnalyticExpansion, PointT, MapT, DrawInterval, DrawStartInterval> {};


#define DEFINE_CONCRETE_ALGORITHM(name, PointT, MapT, Neighborhood, AnalyticExpansion, Heuristic, MapManager, QueueManager) \
    template <int DrawInterval=0, int DrawStartInterval = 0> \
    class name##Search : public GenericSearchAlgorithm<GenericParameter<PointT,Heuristic,MapT,Neighborhood,AnalyticExpansion,QueueManager,MapManager,DrawInterval,DrawStartInterval> > {};\
    template <int DrawInterval, int DrawStartInterval = 0>\
    class name##Search_Debug : public name##Search<DrawInterval, DrawStartInterval> {};

namespace lib_path
{
DEFINE_ALGORITHM(Dijkstra,
                 NoHeuristic,
                 GridMapManager,
                 PriorityQueueManager)

DEFINE_ALGORITHM(DijkstraState,
                 NoHeuristic,
                 DirectionalStateSpaceManager,
                 PriorityQueueManager)

DEFINE_ALGORITHM(AStar2d,
                 HeuristicL2,
                 GridMapManager,
                 PriorityQueueManager)

DEFINE_ALGORITHM(AStar2dTaxi,
                 HeuristicL1,
                 GridMapManager,
                 PriorityQueueManager)

DEFINE_ALGORITHM(AStar2dInf,
                 HeuristicLInf,
                 GridMapManager,
                 PriorityQueueManager)


DEFINE_ALGORITHM(AStar,
                 HeuristicL2,
                 DirectionalStateSpaceManager,
                 PriorityQueueManager)
//typedef MaxHeuristic<HeuristicL2, HeuristicNonHolonomicNoObstacles> MH1;
typedef MaxHeuristic<HeuristicL2, HeuristicHolonomicObstacles> MH1;
//typedef HeuristicL2 MH1;
//typedef HeuristicNonHolonomicNoObstacles MH1;


DEFINE_ALGORITHM(AStarOverEstimate,
                 HeuristicL2OverEstimate,
                 DirectionalStateSpaceManager,
                 PriorityQueueManager)



DEFINE_ALGORITHM(AStarHybridHeuristics,
                 MH1,
                 DirectionalStateSpaceManager,
                 PriorityQueueManager)
}

#endif // BREADTHFIRSTSEARCH_H
