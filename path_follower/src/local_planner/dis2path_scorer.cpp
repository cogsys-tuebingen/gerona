/// HEADER
#include <path_follower/local_planner/dis2path_scorer.h>

Dis2Path_Scorer::Dis2Path_Scorer():
    currentPath()
{
}

Dis2Path_Scorer::~Dis2Path_Scorer()
{

}

void Dis2Path_Scorer::setSubPath(const SubPath& path){
    currentPath.clear();
    currentPath.assign(path.begin(),path.end());
}

double Dis2Path_Scorer::score(const tf::Point& point){
    double closest_dist = std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i < currentPath.size(); ++i) {
        const Waypoint& wp = currentPath[i];
        double dist = std::hypot(wp.x - point.getX(), wp.y - point.getY());
        if(dist < closest_dist) {
            closest_dist = dist;
        }
    }
    return closest_dist;
}
