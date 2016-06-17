/// HEADER
#include <path_follower/local_planner/dis2path_scorer.h>

Dis2Path_Scorer::Dis2Path_Scorer():
    currentPath(), index1_(-1), index2_(-1)
{
    sw.resetStopped();
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

double Dis2Path_Scorer::score(const LNode& point){
    sw.resume();
    double closest_dist = std::numeric_limits<double>::infinity();
    if(currentPath.empty()){
        sw.stop();
        return 0.0;
    }
    for(std::size_t i = index1_; i <= index2_; ++i) {
        const Waypoint& wp = currentPath[i];
        double dist = std::hypot(wp.x - point.x, wp.y - point.y);
        if(dist < closest_dist) {
            closest_dist = dist;
        }
    }
    sw.stop();
    return closest_dist;
}
