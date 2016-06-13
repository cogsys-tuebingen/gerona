/// HEADER
#include <path_follower/local_planner/scorer.h>

Scorer::Scorer()
{

}

Scorer::~Scorer()
{

}

long Scorer::nsUsed(){
    return sw.nsElapsedStatic();
}
