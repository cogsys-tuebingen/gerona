/// HEADER
#include <path_follower/local_planner/dis2start_scorer.h>

Dis2Start_Scorer::Dis2Start_Scorer():
    currentPath(), distances(), index1(-1), index2(-1)
{

}

Dis2Start_Scorer::~Dis2Start_Scorer()
{

}

void Dis2Start_Scorer::setPath(PathInterpolated& iPath, const SubPath& Path){
    currentPath.clear();
    currentPath.assign(Path.begin(),Path.end());
    //For now assumed ds = 1.5
    double s_new = iPath.s_new() + 1.5;
    double closest_dist1 = std::numeric_limits<double>::infinity();
    double closest_dist2 = std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i < iPath.n(); ++i){
        double dist1 = std::abs(iPath.s_new() - iPath.s(i));
        double dist2 = std::abs(s_new - iPath.s(i));
        if(dist1 < closest_dist1) {
            closest_dist1 = dist1;
            index1 = i;
        }
        if(dist2 < closest_dist2) {
            closest_dist2 = dist2;
            index2 = i;
        }
    }

    distances.clear();
    for(std::size_t i = index1; i <= index2; ++i){
        distances.push_back(iPath.s(i));
    }
    iPath.set_s_new(s_new - 0.8);
    /*currentPath.clear();
    currentPath.assign(path.begin(),path.end());
    distances.push_back(0.0);
    for(std::size_t i = 1; i < path.size(); ++i){
        double ndist = distances[distances.size()-1] + path[i-1].distanceTo(path[i]);
        distances.push_back(ndist);
    }*/
}

double Dis2Start_Scorer::score(const tf::Point& point){
    double closest_dist = std::numeric_limits<double>::infinity();
    std::size_t closest_index = -1;

    for(std::size_t i = index1; i <= index2; ++i){
        const Waypoint& wp = currentPath[i];
        double dist = std::hypot(wp.x - point.getX(), wp.y - point.getY());
        if(dist < closest_dist) {
            closest_dist = dist;
            closest_index = i;
        }
    }
    return distances[closest_index-index1];
}
