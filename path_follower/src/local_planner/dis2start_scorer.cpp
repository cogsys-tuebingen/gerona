/// HEADER
#include <path_follower/local_planner/dis2start_scorer.h>

Dis2Start_Scorer::Dis2Start_Scorer():
    distances(),
    currentPath()
{

}

Dis2Start_Scorer::~Dis2Start_Scorer()
{

}

void Dis2Start_Scorer::setDistances(const SubPath& path){
    currentPath.clear();
    currentPath.assign(path.begin(),path.end());
    distances.push_back(0.0);
    for(std::size_t i = 1; i < path.size(); ++i){
        double ndist = distances[distances.size()-1] + path[i-1].distanceTo(path[i]);
        distances.push_back(ndist);
    }
}

double Dis2Start_Scorer::score(const tf::Point& point){
    double closest_dist = std::numeric_limits<double>::infinity();
    std::size_t closest_index = -1;
    for(std::size_t i = 0; i < currentPath.size(); ++i){
        const Waypoint& wp = currentPath[i];
        double dist = std::hypot(wp.x - point.getX(), wp.y - point.getY());
        if(dist < closest_dist) {
            closest_dist = dist;
            closest_index = i;
        }
    }
    return distances[closest_index];
}
