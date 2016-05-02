/// HEADER
#include <path_follower/local_planner/dis2path_constraint.h>

Dis2Path_Constraint::Dis2Path_Constraint()
{
    currentPath.clear();
}

Dis2Path_Constraint::~Dis2Path_Constraint()
{

}

void Dis2Path_Constraint::setSubPath(SubPath &current){
    currentPath = current;
}

bool Dis2Path_Constraint::isSatisfied(const tf::Point &point){

}
