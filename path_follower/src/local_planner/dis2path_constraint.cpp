/// HEADER
#include <path_follower/local_planner/dis2path_constraint.h>

Dis2Path_Constraint::Dis2Path_Constraint():
    currentPath(), index1_(-1), index2_(-1)
{
    sw.resetStopped();
}

Dis2Path_Constraint::~Dis2Path_Constraint()
{

}

void Dis2Path_Constraint::setSubPath(const SubPath& path,
                                     const std::size_t index1, const std::size_t index2){
    currentPath.clear();
    currentPath.assign(path.begin(),path.end());

    index1_ = index1;
    index2_ = index2;
}

bool Dis2Path_Constraint::isSatisfied(const tf::Point& point){
    sw.resume();
    double closest_dist = std::numeric_limits<double>::infinity();
    for(std::size_t i = index1_; i <= index2_; ++i) {
        const Waypoint& wp = currentPath[i];
        double dist = std::hypot(wp.x - point.getX(), wp.y - point.getY());
        if(dist < closest_dist) {
            closest_dist = dist;
            if(closest_dist <= 0.3){
                sw.stop();
                return true;
            }
        }
    }
    sw.stop();
    return false;
}
