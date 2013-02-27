/*
 * Common.hpp
 *
 *  Created on: Feb 06, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef COMMON_H
#define COMMON_H

/// PROJECT
#include <utils/LibGeneric/Utils.hpp>

/// SYSTEM
#include <boost/noncopyable.hpp>
#include <vector>
#include <functional>
#include <math.h>
#include <cstdlib>

namespace lib_path
{

template <class NodeT, class NeighborhoodType>
struct NeighborSelection {};

template <class Type, class PointT>
struct HeuristicSelection {};



template <class PointT>
struct Node : public PointT {
    typedef PointT PointType;
    typedef Node<PointT> NodeType;

    static const char MARK_NONE = 0;
    static const char MARK_OPEN = 1;
    static const char MARK_CLOSED = 2;
    static const char MARK_WATCHED = 4;

    void mark(char mark) {
        marked |= mark;
    }

    bool isMarked(char mark) const {
        return (marked & mark) == mark;
    }

    virtual double getTotalCost() const {
        return distance;
    }

    static void init(Node<PointT> &memory, int x, int y) {
        memory.x = x;
        memory.y = y;
        memory.distance = INFINITY;
        memory.prev = NULL;
        memory.marked = MARK_NONE;
    }

    double distance;
    char marked;
    Node* prev;
};


template <class Node>
struct HybridNode : public Node {
    typedef typename Node::PointType PointType;
    typedef HybridNode<Node> NodeType;

    double center_x;
    double center_y;
    bool forward;

    static void init(HybridNode<Node> &memory, int x, int y, double theta = 0.0) {
        Node::init(memory, x, y);

        memory.center_x = x;
        memory.center_y = y;
        memory.theta = theta;
        memory.forward = true;
    }
};

template <class Node>
struct CompareNode : public std::binary_function<Node*, Node*, bool> {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->getTotalCost() > rhs->getTotalCost();
    }
};



template <class Param>
struct MapManagerExtension {
    typedef typename Param::NodeType NodeType;

    MapManagerExtension()
        : data(NULL), w(0), h(0), theta_slots(1) {
    }

    virtual ~MapManagerExtension() {
        if(data != NULL) {
            delete [] data;
        }
    }

    NodeType* data;
    unsigned w;
    unsigned h;
    unsigned theta_slots;
};



}
#endif // COMMON_H
