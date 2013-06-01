/*
 * DistanceTransformation.hpp
 *
 *  Created on: Apr 18, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef DISTANCETRANSFORMATION_HPP
#define DISTANCETRANSFORMATION_HPP

/// COMPONENT
#include "Common.hpp"

/// PROJECT
#include "SearchAlgorithm.hpp"

namespace lib_path
{

/**
 * @brief The GenericSearchAlgorithm class does all the common work
 */
template <class Param>
class DistanceTransformationSearch :
    public GenericSearchAlgorithm<Param>
{
    typedef GenericSearchAlgorithm<Param> Parent;

public:
    typedef typename Parent::PathT PathT;
    typedef typename Parent::PointT PointT;
    typedef typename Parent::NodeT NodeT;
    typedef typename Parent::Heuristic Heuristic;
    typedef typename Parent::NeighborhoodType NeighborhoodType;
    typedef typename Parent::MapManager MapManager;
    typedef typename Parent::MapT MapT;

    using Parent::expansions;
    using Parent::start;
    using Parent::goal;
    using Parent::map_;

    using Parent::INTERMISSION_N_STEPS;

public:
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
        return findPathImp<generic::CallbackIntermission<INTERMISSION_N_STEPS> >(from, to, intermission);
    }

protected:
    template <class Intermission>
    PathT findPathImp(const PointT& from, const PointT& to, boost::function<void()> intermission) {
        expansions = 0;

#define LOOKUP(a,b) map_.lookup(a,b)

        // initialize start and goal nodes
        start = map_.lookup(from);
        goal = map_.lookup(to);

        int w = map_.w;
        int h = map_.h;

        for(int row = 0; row < h; ++row) {
            for(int col = 0; col < w; ++col) {
                if(map_.isFree(col, row)) {
                    LOOKUP(col, row)->distance = std::numeric_limits<double>::max();
                } else {
                    LOOKUP(col, row)->distance = INFINITY;
                }
            }
        }

        LOOKUP(to.x, to.y)->distance = 0;

        bool change = true;
        while(change) {
            change = false;
            expansions++;

            for(int row = 1; row < h-1; ++row) {
                for(int col = 1; col < w-1; ++col) {
                    NodeT* node = LOOKUP(col, row);

                    double& d = node->distance;
                    bool free = d != INFINITY;

                    if(free) {
                        double d1 = LOOKUP(col-1, row  )->distance + 1;
                        double d2 = LOOKUP(col-1, row-1)->distance + sqrt(2);
                        double d3 = LOOKUP(col,   row-1)->distance + 1;
                        double d4 = LOOKUP(col+1, row-1)->distance + sqrt(2);

                        double dmin = std::min(d1, std::min(d2, std::min(d3, d4)));

                        if(dmin < d) {
                            d = dmin;
                            change = true;
                            node->mark(NodeT::MARK_EXPANDED);
                        }
                        node->mark(NodeT::MARK_WATCHED);
                        Intermission::call(intermission);
                    }
                }
            }

//            Intermission::call(intermission);

            for(int row = h-2; row >= 1; --row) {
                for(int col = w-2; col >= 1; --col) {
                    NodeT* node = LOOKUP(col, row);

                    double& d = node->distance;
                    bool free = d != INFINITY;

                    if(free) {
                        double d1 = LOOKUP(col+1, row  )->distance + 1;
                        double d2 = LOOKUP(col-1, row+1)->distance + sqrt(2);
                        double d3 = LOOKUP(col,   row+1)->distance + 1;
                        double d4 = LOOKUP(col+1, row+1)->distance + sqrt(2);

                        double dmin = std::min(d1, std::min(d2, std::min(d3, d4)));

                        if(dmin < d) {
                            d = dmin;
                            change = true;
                            node->mark(NodeT::MARK_EXPANDED);
                        }
                        node->mark(NodeT::MARK_WATCHED);
                        Intermission::call(intermission);
                    }
                }
            }

//            Intermission::call(intermission);
        }

        return backtrack(start, goal);
    }

    PathT backtrack(NodeT* start, NodeT* goal) {
        PathT path;

        NodeT* current = start;
        path.push_back(*start);

        while(current != goal) {
            NodeT* best = current;
            for(int i = 0; i < NeighborhoodType::SIZE; ++i) {
                int xx = NeighborhoodType::dx(current->x,i);
                int yy = NeighborhoodType::dy(current->y,i);

                NodeT* c = LOOKUP(xx, yy);
                double tmp = c->distance;
                if(tmp < best->distance) {
                    best = c;
                }
            }

            if(best == current) {
                std::cout << "done, no path found" << std::endl;
                return path;
            }

            path.push_back(*best);

            current = best;
        }

        std::cout << "found path" << std::endl;
        return path;
    }

#undef LOOKUP
};

}


#endif // DISTANCETRANSFORMATION_HPP
