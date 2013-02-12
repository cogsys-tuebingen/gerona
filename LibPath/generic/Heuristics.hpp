/*
 * Heuristics.hpp
 *
 *  Created on: 2 12, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

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




template <class PointT>
struct NoHeuristic {
    typedef Node<PointT> NodeType;

    template <class NodeType>
    static void compute(const NodeType*, const NodeType*) {
    }
};

template <class PointT>
struct HeuristicDistToGoal {
    typedef HeuristicNode<PointT> NodeType;

    template <class NodeType>
    static void compute(NodeType* current, NodeType* goal) {
        current->h = hypot(current->x - goal->x, current->y - goal->y);
    }
};


}

#endif // HEURISTICS_HPP
