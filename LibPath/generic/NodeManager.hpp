/*
 * NodeManager.hpp
 *
 *  Created on: Feb 02, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef NODEMANAGER_H
#define NODEMANAGER_H

/// COMPONENT
#include "Common.hpp"

/// SYSTEM
#include <queue>
#include <stack>

namespace lib_path
{

template <class Node>
struct QueueManager {
    void add(Node* node) {
        queue.push_back(node);
    }

    bool empty() {
        return queue.empty();
    }

    Node* next() {
        assert(!queue.empty());
        Node* next = queue.front();
        queue.pop_front();
        return next;
    }

private:
    std::deque<Node*> queue;
};

template <class Node, class Container>
struct GenericManager {
    void add(Node* node) {
        container.push(node);
    }

    bool empty() {
        return container.empty();
    }

    Node* next() {
        assert(!container.empty());
        Node* next = container.top();
        container.pop();
        return next;
    }

private:
    Container container;
};


template <class Node>
class PriorityQueueManager : public GenericManager<Node, std::priority_queue<Node*, std::vector<Node*>, CompareNode<Node> > >
{
};
template <class Node>
class StackManager : public GenericManager<Node, std::stack<Node*> >
{
};

}

#endif // NODEMANAGER_H
