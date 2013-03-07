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

/**
 * @brief The NoSubParameter struct is a dummy class
 */
struct NoSubParameter {};


/**
 * @brief The GenericParameter struct combines types to be used by the search algorithm
 */
template <class PointT,
         class HeuristicT,
         class MapT,
         class NeighborhoodT,
         template <class> class OpenNodesPolicy,
         int INTERMISSION_N_STEPS = 0,
         class SubParameter = NoSubParameter
         >
struct GenericParameter : public SubParameter
{
    typedef PointT PointType;
    typedef HeuristicT HeuristicType;
    typedef MapT MapType;
    typedef NeighborhoodT NeighborhoodType;

    // NeighborhoodSelection allows to augment Nodes depending on the selected neighborhood
    typedef NeighborhoodPolicy<typename HeuristicType::template NodeHolder<PointT>::NodeType, NeighborhoodType> NeighborhoodSelectionPolicy;
    typedef typename NeighborhoodSelectionPolicy::NodeType NodeType;

    typedef OpenNodesPolicy<NodeType> OpenNodesManager;

    enum { INTERMISSION_STEPS = INTERMISSION_N_STEPS};
};


/**
 * @brief The GenericSearchAlgorithm class does all the common work
 */
template <class Param,
         template <class, class> class MapManagerPolicy,
         template <class> class Extension = MapManagerExtension>
class GenericSearchAlgorithm :
    public Param::NeighborhoodSelectionPolicy,
    public MapManagerPolicy<Param, Extension<Param> >
{
public:
    enum { INTERMISSION_N_STEPS = Param::INTERMISSION_STEPS };
    enum { SCALE = Param::SCALE };

    typedef typename Param::OpenNodesManager OpenNodesManager;
    typedef typename Param::PointType PointT;
    typedef typename Param::NodeType NodeT;
    typedef typename Param::HeuristicType Heuristic;

    friend class Param::NeighborhoodSelectionPolicy;

    typedef GenericPath<NodeT> PathT;

public:
    /**
     * @brief empty
     * @return an empty path
     */
    static const PathT empty() {
        return PathT();
    }

    /**
     * @brief noExpansions
     * @return the number of expansions used
     */
    double noExpansions() const {
        return expansions;
    }

    /**
     * @brief findPath searches a path between to points
     * @param from
     * @param to
     * @return
     */
    PathT findPath(const PointT& from, const PointT& to) {
        return findPathImp<generic::NoIntermission>(from, to, boost::function<void()>());
    }

    /**
     * @brief findPath searches a path between to points
     * @param from
     * @param to
     * @param intermission is a callback that is called every INTERMISSION_N_STEPS steps
     * @return
     */
    PathT findPath(const PointT& from, const PointT& to, boost::function<void()> intermission) {
        return findPathImp<generic::CallbackIntermission<INTERMISSION_N_STEPS> >(from, to, intermission);
    }

private:
    template <class Intermission>
    PathT findPathImp(const PointT& from, const PointT& to, boost::function<void()> intermission) {
        expansions = 0;

        // initialize start and goal nodes
        start = lookup(from);
        goal = lookup(to);

        // search can be aborted, if eather one is occupied
        if(!isFree(start) || !isFree(goal)) {
            return empty();
        }

        // else init the border cases
        NodeT::init(*start, from);
        NodeT::init(*goal, to);

        // put start node in open data structure
        start->distance = 0;
        start->mark(NodeT::MARK_OPEN);

        open.add(start);

        // iterate until no more points can be looked at
        while(!open.empty()) {
            // mark the next node closed
            NodeT* current = open.next();
            current->mark(NodeT::MARK_CLOSED);

            // if we are close enough to the goal (policy decides), we can return
            if(isNearEnough(goal, current)) {
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
            iterateFreeNeighbors(*this, current);

            // inermission, if used
            Intermission::call(intermission);
        }

        std::cout << "done, no path found" << std::endl;

        // default: empty path
        return empty();
    }


    bool processNeighbor(NodeT* current, NodeT* neighbor, double delta) {
        if(neighbor->isMarked(NodeT::MARK_CLOSED)) {
            return false;
        }

        neighbor->mark(NodeT::MARK_WATCHED);

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

private:
    OpenNodesManager open;
    NodeT* start;
    NodeT* goal;

    int expansions;
};

}

#endif // SEARCH_ALGORITHM_H
