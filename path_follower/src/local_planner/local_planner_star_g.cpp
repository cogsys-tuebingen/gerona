/// HEADER
#include <path_follower/local_planner/local_planner_star_g.h>

/// PROJECT


LocalPlannerStarG::LocalPlannerStarG(RobotController &controller, PoseTracker &pose_tracker,
                                     const ros::Duration &update_interval)
    : LocalPlannerClassic(controller,pose_tracker,update_interval),
      LocalPlannerStar(controller,pose_tracker,update_interval)
{

}

double LocalPlannerStarG::f(double& g, double& score, double& heuristic){
    (void) g;
    return score + heuristic;
}
