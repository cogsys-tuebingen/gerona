/*
 * Common.hpp
 *
 *  Created on: Feb 06, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef COMMON_H
#define COMMON_H

/// PROJECT
#include <LibGeneric/Utils.hpp>

/// SYSTEM
#include <boost/noncopyable.hpp>
#include <vector>
#include <functional>
#include <math.h>

namespace lib_path {

template <class Neighborhood, class MapManager, class NodeT, class MapT, class NoType>
struct NeighborSelection {};

template <class NodeT, class MapT, class NoType>
class MapManagerSelection {};

template <class Type, class PointT>
struct HeuristicSelection {};



template <class PointT>
struct Node : public PointT, boost::noncopyable {
    void mark() {
        marked = true;
    }

    bool isMarked() const {
        return marked;
    }

    virtual double getTotalCost() const {
        return distance;
    }

    double distance;
    bool marked;
    Node * prev;
    double delta;
};

template <class PointT>
struct HeuristicNode : public Node<PointT> {
    using Node<PointT>::distance;

    virtual double getTotalCost() const {
        return distance + h;
    }

    double h;
};

template <class Node>
struct CompareNode : public std::binary_function<Node*, Node*, bool>
{
    bool operator()(const Node* lhs, const Node* rhs) const
    {
        return lhs->getTotalCost() > rhs->getTotalCost();
    }
};

class NoHeuristic {};

template <class PointT>
struct HeuristicSelection<NoHeuristic, PointT> : public boost::noncopyable {
    typedef Node<PointT> NodeType;

    static void compute(...) {
    }
};


class HeuristicDistToGoal {};

template <class PointT>
struct HeuristicSelection<HeuristicDistToGoal, PointT> : public  boost::noncopyable {
    typedef HeuristicNode<PointT> NodeType;

    static void compute(NodeType* current, NodeType* goal) {
        current->h = hypot(current->x - goal->x, current->y - goal->y);
    }
};

}
#endif // COMMON_H
