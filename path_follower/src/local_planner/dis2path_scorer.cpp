/// HEADER
#include <path_follower/local_planner/dis2path_scorer.h>

Dis2Path_Scorer::Dis2Path_Scorer():
    currentPath(), index1_(-1), index2_(-1)
{
}

Dis2Path_Scorer::~Dis2Path_Scorer()
{

}

void Dis2Path_Scorer::setSubPath(const SubPath& path,
                                 const std::size_t index1, const std::size_t index2){
    currentPath.clear();
    currentPath.assign(path.begin(),path.end());

    index1_ = index1;
    index2_ = index2;
}

double Dis2Path_Scorer::score(const tf::Point& point){
    double closest_dist = std::numeric_limits<double>::infinity();
    if(currentPath.empty()){
        return 0.0;
    }
    for(std::size_t i = index1_; i <= index2_; ++i) {
        const Waypoint& wp = currentPath[i];
        double dist = std::hypot(wp.x - point.getX(), wp.y - point.getY());
        if(dist < closest_dist) {
            closest_dist = dist;
        }
    }
    return closest_dist;
}
