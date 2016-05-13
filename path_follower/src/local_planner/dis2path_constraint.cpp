/// HEADER
#include <path_follower/local_planner/dis2path_constraint.h>

Dis2Path_Constraint::Dis2Path_Constraint():
    currentPath()
{
}

Dis2Path_Constraint::~Dis2Path_Constraint()
{

}

void Dis2Path_Constraint::setSubPath(const SubPath& path){
    currentPath.clear();
    currentPath.assign(path.begin(),path.end());
}

bool Dis2Path_Constraint::isSatisfied(const tf::Point& point){
    double closest_dist = std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i < currentPath.size(); ++i) {
        const Waypoint& wp = currentPath[i];
        double dist = std::hypot(wp.x - point.getX(), wp.y - point.getY());
        if(dist < closest_dist) {
            closest_dist = dist;
            if(closest_dist <= 0.22){
                return true;
            }
        }
    }
    return false;
}
