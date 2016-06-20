/// HEADER
#include <path_follower/local_planner/dis2start_scorer.h>

Dis2Start_Scorer::Dis2Start_Scorer():
    Scorer(),currentPath(), distances(), index1_(-1), index2_(-1)
{

}

Dis2Start_Scorer::~Dis2Start_Scorer()
{

}

void Dis2Start_Scorer::setPath(const SubPath& Path, const std::vector<double>& dists,
                               const std::size_t index1, const std::size_t index2){
    currentPath.clear();
    currentPath.assign(Path.begin(),Path.end());

    distances.clear();
    distances.assign(dists.begin(),dists.end());

    index1_ = index1;
    index2_ = index2;
}

double Dis2Start_Scorer::score(const LNode& point){
    sw.resume();
    double closest_dist = std::numeric_limits<double>::infinity();
    std::size_t closest_index = -1;

    for(std::size_t i = index1_; i <= index2_; ++i){
        const Waypoint& wp = currentPath[i];
        double dist = std::hypot(wp.x - point.x, wp.y - point.y);
        if(dist < closest_dist) {
            closest_dist = dist;
            closest_index = i;
        }
    }
    sw.stop();
    return distances[closest_index-index1_];
}
