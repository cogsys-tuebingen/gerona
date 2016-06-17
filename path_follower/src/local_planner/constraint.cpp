/// HEADER
#include <path_follower/local_planner/constraint.h>

Constraint::Constraint()
{

}

Constraint::~Constraint()
{

}

long Constraint::nsUsed(){
    return sw.nsElapsedStatic();
}
