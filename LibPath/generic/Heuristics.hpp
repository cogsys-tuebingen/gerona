/*
 * Heuristics.hpp
 *
 *  Created on: 2 12, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

/// SYSTEM
#include <boost/static_assert.hpp>

namespace lib_path
{

template <class PointT>
struct HeuristicNode : public Node<PointT> {
    using Node<PointT>::distance;

    typedef PointT PointType;
    typedef HeuristicNode<PointT> NodeType;

    virtual double getTotalCost() const {
        return distance + h;
    }

    static void init(HeuristicNode<PointT> &memory, int x, int y) {
        Node<PointT>::init(memory, x, y);
        memory.h = 0;
    }

    double h;
};




struct NoHeuristic {
    template <class PointT>
    struct NodeHolder {
        typedef Node<PointT> NodeType;
    };

    template <class NodeType>
    static void compute(const NodeType*, const NodeType*) {
    }
};

struct HeuristicDistToGoal {
    template <class PointT>
    struct NodeHolder {
        typedef HeuristicNode<PointT> NodeType;
    };

    template <class NodeType>
    static void compute(NodeType* current, NodeType* goal) {
        current->h = hypot(current->x - goal->x, current->y - goal->y);
    }
};

template <class H1, class H2>
struct MaxHeuristic
{
    template <class PointT>
    struct NodeHolder {
        typedef typename H1::template NodeHolder<PointT> H1NH;
        typedef typename H1NH::NodeType T1;

        typedef typename H2::template NodeHolder<PointT> H2NH;
        typedef typename H2NH::NodeType T2;

        BOOST_STATIC_ASSERT((boost::is_same<T1, T2>::value));

        typedef T1 NodeType;
    };

    template <class NodeType>
    static void compute(NodeType* current, NodeType* goal) {
        H1::compute(current, goal);
        double h2 = current->h;

        H2::compute(current, goal);

        current->h = std::max(current->h, h2);
    }
};


}

#endif // HEURISTICS_HPP
