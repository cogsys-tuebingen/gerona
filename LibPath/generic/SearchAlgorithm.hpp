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
#include <utils/LibGeneric/Intermission.hpp>
#include "../common/Path.h"

/// SYSTEM
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>

namespace lib_path
{

struct NoSubParameter {
    enum { SCALE = 2};
    typedef int Connector;
};

template <class PointT,
         class HeuristicT,
         class MapT,
         class Neighborhood,
         template <class> class OpenNodesManager,
         int DRAW_N_STEPS = 0,
         class SubParameter = NoSubParameter
         >
struct GenericParameter : public SubParameter
{
    typedef PointT PointType;
    typedef HeuristicT HeuristicType;
    typedef MapT MapType;
    typedef Neighborhood NeighborhoodType;
    typedef NeighborSelection<typename HeuristicType::template NodeHolder<PointT>::NodeType, NeighborhoodType> NeighborhoodSelection;
    typedef typename NeighborhoodSelection::NodeType NodeType;

    typedef OpenNodesManager<NodeType> OpenNodesManagerType;

    enum { DRAW_STEPS = DRAW_N_STEPS};
};

template <class Param,
         template <class, class> class MapManager,
         template <class> class Extension = MapManagerExtension>
class GenericSearchAlgorithm :
    public Param::NeighborhoodSelection,
    public MapManager<typename Param::NodeType, Extension<Param> >
{
public:

    enum { DRAW_N_STEPS = Param::DRAW_STEPS };
    enum { SCALE = Param::SCALE };

    typedef typename Param::OpenNodesManagerType OpenNodesManager;
    typedef typename Param::PointType PointT;
    typedef typename Param::NodeType NodeT;
    typedef typename Param::HeuristicType Heuristic;
//    typedef NeighborSelection<Neighborhood, MapManager, typename Heuristic::NodeType, MapT, Extension> NeighborhoodT;

    typedef GenericPath<NodeT> PathT;

    static const PathT empty() {
        return PathT();
    }

    double noExpansions() const {
        return expansions;
    }

    PathT findPath(const PointT& from, const PointT& to) {
        return findPathImp<generic::NoIntermission>(from, to, boost::function<void()>());
    }
    PathT findPath(const PointT& from, const PointT& to, boost::function<void()> intermission) {
        return findPathImp<generic::CallbackIntermission<DRAW_N_STEPS> >(from, to, intermission);
    }

private:
    template <class Intermission>
    PathT findPathImp(const PointT& from, const PointT& to, boost::function<void()> intermission) {
        expansions = 0;

        start = lookup(from);
        goal = lookup(to);

        if(!isFree(start) || !isFree(goal)) {
            return empty();
        }

        start->theta = from.theta;
        goal->theta = to.theta;

        start->distance = 0;
        start->mark(NodeT::MARK_OPEN);

        open.add(start);

        while(!open.empty()) {
            NodeT* current = open.next();
            current->mark(NodeT::MARK_CLOSED);

            if(isNearEnough(goal, current)) {
                if(goal != current) {
                    goal->prev = current;
                    std::cout << "near goal: theta=" << current->theta << ", goal=" << goal->theta << std::endl;
                }
                std::cout << "found goal: theta=" << current->theta << ", goal=" << goal->theta << std::endl;
                return backtrack(start, goal);
            }

            iterateFreeNeighbors(this, current);

            Intermission::call(intermission);
        }

        std::cout << "done, no path found" << std::endl;

        return empty();
    }

    bool forEachFreeNeighbor(NodeT* current, NodeT* neighbor, double delta) {
        if(neighbor->isMarked(NodeT::MARK_CLOSED)) {
            return false;
        }

        double distance = current->distance + delta;
        bool closer = distance < neighbor->distance;
        bool notInOpenList = !neighbor->isMarked(NodeT::MARK_OPEN);

        if(notInOpenList) {
            open.add(neighbor);
            neighbor->mark(NodeT::MARK_OPEN);

            expansions ++;
        }

        if(closer) {
            Heuristic::compute(neighbor, goal);

            neighbor->distance = distance;

            assert(neighbor != current);
            neighbor->prev = current;

            return true;
        }

        return false;
    }

    PathT backtrack(NodeT* start, NodeT* goal) {
        PathT path;

        NodeT* current = goal;

        while(current != start) {
            assert(current != NULL);
            assert(current != current->prev);
            path.push_back(*current);
            current = dynamic_cast<NodeT*>(current->prev);
        }
        path.push_back(*start);

        std::reverse(path.begin(), path.end());

        return path;
    }

    OpenNodesManager open;
    NodeT* start;
    NodeT* goal;

    int expansions;
};

}

#endif // SEARCH_ALGORITHM_H
