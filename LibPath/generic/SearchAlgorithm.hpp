/*
 * SearchAlgorithm.hpp
 *
 *  Created on: Feb 01, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef SEARCH_ALGORITHM_H
#define SEARCH_ALGORITHM_H

/// COMPONENT
#include "Common.hpp"

/// PROJECT
#include <LibGeneric/Intermission.hpp>
#include "../common/Path.h"

/// SYSTEM
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>

namespace lib_path
{
template <class PointT,
          class HeuristicType,
          class Neighborhood,
          class MapT,
          class NeighborType,
          class MapManager,
          template <class> class OpenNodesManager,
          int DRAW_N_STEPS = 0
          >
class GenericSearchAlgorithm :
        public NeighborSelection<Neighborhood, MapManager, typename HeuristicSelection<HeuristicType, PointT>::NodeType, MapT, NeighborType>
{
public:
    static const GenericPath<PointT> empty() {
        return GenericPath<PointT>();
    }

    typedef HeuristicSelection<HeuristicType, PointT> Heuristic;
    typedef typename Heuristic::NodeType NodeT;

    GenericPath<PointT> findPath(const PointT& from, const PointT& to)
    {
        return findPathImp<generic::NoIntermission>(from, to, boost::function<void()>());
    }
    GenericPath<PointT> findPath(const PointT& from, const PointT& to, boost::function<void()> intermission)
    {
        return findPathImp<generic::CallbackIntermission<DRAW_N_STEPS> >(from, to, intermission);
    }

private:
    template <class Intermission>
    GenericPath<PointT> findPathImp(const PointT& from, const PointT& to, boost::function<void()> intermission)
    {
        start = lookup(from);
        goal = lookup(to);

        if(!isFree(start) || !isFree(goal)){
            return empty();
        }

        start->distance = 0;
        start->mark();

        open.add(start);

        while(!open.empty()) {
            NodeT* current = open.next();

            if(isNearEnough(goal, current)){
                goal->prev = current;
                return backtrack(start, goal);
            }

            iterateFreeNeighbors(current);

            Intermission::call(intermission);
        }

        std::cout << "done, no path found" << std::endl;

        return empty();
    }

    void forEachFreeNeighbor(NodeT* current, NodeT* neighbor){
        double distance = current->distance + neighbor->delta;
        bool closer = distance < neighbor->distance;
        if(!neighbor->isMarked() || closer) {
            open.add(neighbor);
            neighbor->mark();

            Heuristic::compute(neighbor, goal);

            if(closer) {
                neighbor->distance = distance;
                neighbor->prev = current;
            }
        }
    }

    GenericPath<PointT> backtrack(NodeT* start, NodeT* goal)
    {
        GenericPath<PointT> path;

        Node<PointT>* current = goal;

        while(current != start) {
            path.push_back(*current);
            current = current->prev;
        }
        path.push_back(*start);

        std::reverse(path.begin(), path.end());

        return path;
    }

    OpenNodesManager<NodeT > open;
    NodeT* start;
    NodeT* goal;
};

}

#endif // SEARCH_ALGORITHM_H
