#ifndef SEGMENT_H
#define SEGMENT_H

#include <cslibs_path_planning/geometry/line.h>
#include "transition.h"

class Segment
{
public:
    path_geom::Line line;
    std::vector<Transition> forward_transitions;
    std::vector<Transition> backward_transitions;

    Segment(const path_geom::Line& line);
};

#endif // SEGMENT_H
