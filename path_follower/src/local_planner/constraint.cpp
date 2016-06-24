/// HEADER
#include <path_follower/local_planner/constraint.h>

Constraint::Constraint()
{
    sw.resetStopped();
}

Constraint::~Constraint()
{

}

long Constraint::nsUsed(){
    return sw.nsElapsedStatic();
}
