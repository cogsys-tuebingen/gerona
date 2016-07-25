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

    if(closest_index == index1_){
        const Waypoint& p0 = currentPath[0];
        const Waypoint& p1 = currentPath[1];
        double x = p1.x - p0.x;
        double y = p1.y - p0.y;
        double a_next = atan2(y,x);
        x = point.x - p0.x;
        y = point.y - p0.y;
        double a_point = atan2(y,x);
        double adiff = a_next - a_point;
        adiff += (adiff > M_PI) ? - 2.0*M_PI : (adiff < -M_PI) ? 2.0*M_PI : 0;
        adiff = abs(adiff);
        ROS_INFO_STREAM("adiff = " << adiff);
        if(adiff > M_PI_2){
            double dis = std::hypot(p0.x - point.x, p0.y - point.y);
            sw.stop();
            return distances[closest_index-index1_] - dis;
        }
    }
    sw.stop();
    return distances[closest_index-index1_];
}
