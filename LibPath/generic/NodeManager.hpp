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

template <class Node>
struct PriorityQueueManager {
    void add(Node* node) {
        queue.push(node);
    }

    bool empty() {
        return queue.empty();
    }

    Node* next() {
        assert(!queue.empty());
        Node* next = queue.top();
        queue.pop();
        return next;
    }

private:
    std::priority_queue<Node*, std::vector<Node*>, CompareNode<Node> > queue;
};

template <class Node>
struct StackManager {
    void add(Node* node) {
        stack.push(node);
    }

    bool empty() {
        return stack.empty();
    }

    Node* next() {
        assert(!stack.empty());
        Node* next = stack.top();
        stack.pop();
        return next;
    }

private:
    std::stack<Node*> stack;
};

}

#endif // NODEMANAGER_H
