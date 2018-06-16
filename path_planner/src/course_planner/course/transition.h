#ifndef TRANSITION_H
#define TRANSITION_H

#include <Eigen/Core>
#include <vector>

class Segment;

class Transition {
public:
    double arc_length() const;

    const Segment* source;
    const Segment* target;

    Eigen::Vector2d intersection;
    Eigen::Vector2d icr;

    double r;
    double dtheta;

    std::vector<Eigen::Vector2d> path;
};

#endif // TRANSITION_H
