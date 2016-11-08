/// HEADER
#include <path_follower/local_planner/scorer.h>

Scorer::Scorer()
    : weight_(1.0)
{
    sw.resetStopped();
}

Scorer::~Scorer()
{

}

long Scorer::nsUsed(){
    return sw.nsElapsedStatic();
}

double Scorer::getWeight() const
{
    return weight_;
}
void Scorer::setWeight(double weight)
{
    weight_ = weight;
}

double Scorer::calculateScore(const LNode& point)
{
    return weight_ * score(point);
}
