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
#include "Heuristics.hpp"

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

/**
 * @brief The GenericParameter struct combines types to be used by the search algorithm
 */
template <class PointT,
          class HeuristicT,
          class MapT,
          class NeighborhoodT,
          class AnalyticExpansionT,
          template <class> class OpenNodesPolicy,
          template <class> class MapManagerPolicy,
          int INTERMISSION_N_STEPS = 0,
          int INTERMISSION_START_STEPS = 0
          >
struct GenericParameter
{
    typedef PointT PointType;
    typedef HeuristicT HeuristicType;
    typedef MapT MapType;
    typedef NeighborhoodT NeighborhoodType;
    typedef AnalyticExpansionT AnalyticExpansionType;

private:
    typedef typename HeuristicType::template NodeHolder<PointT>::NodeType HeuristicNodeType;
    typedef typename NeighborhoodType::template NodeHolder<HeuristicNodeType>::NodeType NeighborhoodNodeType;
    typedef typename MapManagerPolicy<NeighborhoodNodeType>::template NodeHolder<NeighborhoodNodeType>::NodeType MapNodeType;

public:
    typedef MapNodeType NodeType;

    typedef MapManagerPolicy<NodeType> MapManager;
    typedef OpenNodesPolicy<NodeType> OpenNodesManager;

    enum { INTERMISSION_STEPS = INTERMISSION_N_STEPS, INTERMISSION_START = INTERMISSION_START_STEPS};
};


/**
 * @brief The GenericSearchAlgorithm class does all the common work
 */
template <class Param>
class GenericSearchAlgorithm
{
public:
    enum { INTERMISSION_N_STEPS = Param::INTERMISSION_STEPS };
    enum { INTERMISSION_START_STEPS = Param::INTERMISSION_START };

    typedef typename Param::OpenNodesManager OpenNodesManager;
    typedef typename Param::PointType PointT;
    typedef typename Param::NodeType NodeT;
    typedef typename Param::HeuristicType Heuristic;
    typedef typename Param::NeighborhoodType NeighborhoodType;
    typedef typename Param::MapManager MapManager;
    typedef typename Param::AnalyticExpansionType AnalyticExpansionType;

    typedef typename MapManager::MapT MapT;

    typedef GenericPath<NodeT> PathT;

public:
    GenericSearchAlgorithm()
        : start(NULL), goal(NULL), expansions(0), multi_expansions(0), updates(0)
    {
    }

    /**
     * @brief empty
     * @return an empty path
     */
    static const PathT empty() {
        return PathT();
    }

    virtual void setMap(const MapT* map) {
        map_.setMap(map);
        NeighborhoodType::setResolution(map->getResolution());
    }

    virtual void setStart(const PointT& from) {
        assert(map_.getMap() != NULL);
        start = map_.lookup(from);
    }

    virtual void setGoal(const PointT& to) {
        assert(map_.getMap() != NULL);
        goal = map_.lookup(to);
    }

    /**
     * @brief findPath searches a path between to points
     * @param from
     * @param to
     * @return
     */
    virtual PathT findPath(const PointT& from, const PointT& to) {
        return findPathImp<generic::NoIntermission>(from, to, boost::function<void()>());
    }

    /**
     * @brief findPath searches a path between to points
     * @param from
     * @param to
     * @param intermission is a callback that is called every INTERMISSION_N_STEPS steps
     * @return
     */
    virtual PathT findPath(const PointT& from, const PointT& to, boost::function<void()> intermission) {
        //return findPathImp<generic::CallbackIntermission<INTERMISSION_N_STEPS> >(from, to, intermission);
        return findPathImp<generic::CallbackIntermission<INTERMISSION_N_STEPS, INTERMISSION_START_STEPS> >(from, to, intermission);
    }

protected:
    template <class Intermission>
    PathT findPathImp(const PointT& from, const PointT& to, boost::function<void()> intermission) {
        expansions = 0;
        multi_expansions = 0;
        updates = 0;

        open.clear();

        // initialize start and goal nodes
        start = map_.lookup(from);
        goal = map_.lookup(to);

        Heuristic::setMap(map_.getMap(), *goal);

        // search can be aborted, if eather one is occupied
        if(!map_.isFree(start)) {
            std::cout << "start cell is not free" << std::endl;
        }
        if(!map_.isFree(goal)) {
            std::cout << "goal cell is not free" << std::endl;
        }
        if(!map_.isFree(start) || !map_.isFree(goal)) {
            return empty();
        }

        // else init the border cases
        NodeT::init(*start, from);
        NodeT::init(*goal, to);

        // put start node in open data structure
        start->distance = 0;
        start->mark(NodeT::MARK_OPEN);

        open.add(start);

        Intermission::call(intermission);

        // iterate until no more points can be looked at
        while(!open.empty()) {
            // mark the next node closed
            NodeT* current = open.next();
            current->unMark(NodeT::MARK_OPEN);
            current->mark(NodeT::MARK_CLOSED);

            Heuristic::compute(current, goal, map_.getMap()->getResolution());

            expansions ++;
            if(current->isMarked(NodeT::MARK_EXPANDED)) {
                ++multi_expansions;
            }
            current->mark(NodeT::MARK_EXPANDED);

            if(AnalyticExpansionType::canExpand(current, goal, map_.getMap())) {
                std::cout << "found an expansion: theta=" << current->theta << ", goal=" << goal->theta << std::endl;

                PathT expansion;
                AnalyticExpansionType::getPath(&expansion);

                current->theta = expansion.begin()->theta;

                PathT path = backtrack(start, current);
                path += expansion;
                return path;
            }

            // if we are close enough to the goal (policy decides), we can return
            if(NeighborhoodType::isNearEnough(goal, current)) {
                if(goal != current) {
                    // close, but not the real goal -> change prev pointer
                    goal->prev = current;
                    std::cout << "near goal: theta=" << current->theta << ", goal=" << goal->theta << std::endl;
                }
                std::cout << "found goal: theta=" << current->theta << ", goal=" << goal->theta << std::endl;

                // generate the path
                return backtrack(start, goal);
            }

            // look at every free neighbor
            NeighborhoodType::iterateFreeNeighbors(*this, map_, current);

            // intermission, if used
            Intermission::call(intermission);
        }

        std::cout << "done, no path found" << std::endl;

        // default: empty path
        return empty();
    }

public:
    NeighborhoodBase::ProcessingResult
    processNeighbor(NodeT* current, NodeT* neighbor, double delta) {

        neighbor->mark(NodeT::MARK_WATCHED);

        double res = map_.getResolution();
        double distance = current->distance + delta * res;
        bool closer = distance < neighbor->distance;

        bool inOpenList = neighbor->isMarked(NodeT::MARK_OPEN);
        bool inClosedList = neighbor->isMarked(NodeT::MARK_CLOSED);

        if(inClosedList && !closer) {
            return NeighborhoodBase::PR_CLOSED;
        }

        if(!inOpenList || closer) {
            assert(neighbor != current);
            if(inOpenList) {
//                open.remove(neighbor);
//                neighbor->unMark(NodeT::MARK_OPEN);
                updates++;
            }

            neighbor->prev = current;

            neighbor->distance = distance;

            Heuristic::compute(neighbor, goal, res);

            neighbor->mark(NodeT::MARK_OPEN);

            open.add(neighbor);

            return NeighborhoodBase::PR_ADDED_TO_OPEN_LIST;
        }

        return NeighborhoodBase::PR_IGNORED;
    }

    PathT backtrack(NodeT* start, NodeT* goal) {
        PathT path;

        NodeT* current = goal;
        path.push_back(*goal);

        while(current != start){
            current = dynamic_cast<NodeT*>(current->prev);
            assert(current != NULL);
            assert(current != current->prev);
            path.push_back(*current);
        };

        std::reverse(path.begin(), path.end());

        return path;
    }

protected:
    OpenNodesManager open;
    MapManager map_;
    NodeT* start;
    NodeT* goal;

    int expansions;
    int multi_expansions;
    int updates;
};

}

#endif // SEARCH_ALGORITHM_H
