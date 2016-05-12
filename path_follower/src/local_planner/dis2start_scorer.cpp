/// HEADER
#include <path_follower/local_planner/dis2start_scorer.h>

Dis2Start_Scorer::Dis2Start_Scorer():
    distances()
{

}

Dis2Start_Scorer::~Dis2Start_Scorer()
{

}

void Dis2Start_Scorer::setDistances(const SubPath& path){
    distances.push_back(0.0);
    for(std::size_t i = 1; i < path.size(); ++i){
        double ndist = distances[distances.size()-1] + path[i-1].distanceTo(path[i-1]);
        distances.push_back(ndist);
    }
}

double Dis2Start_Scorer::score(const tf::Point& point){
    return 1;
}
