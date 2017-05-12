#ifndef NODE_H
#define NODE_H

#include <limits>

class Transition;
class Segment;

struct Node {
    // associated transition
    const Transition* transition = nullptr;

    const Segment* next_segment = nullptr;

    bool curve_forward= true;

    // distance travelled until this transition is reached
    double cost = std::numeric_limits<double>::infinity();

    // node via which this transition is reached
    Node* prev = nullptr;
    Node* next = nullptr;
};

#endif // NODE_H
