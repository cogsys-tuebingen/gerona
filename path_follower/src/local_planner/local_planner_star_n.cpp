/// HEADER
#include <path_follower/local_planner/local_planner_star_n.h>

/// PROJECT


LocalPlannerStarN::LocalPlannerStarN(RobotController &controller, PoseTracker &pose_tracker,
                                     const ros::Duration &update_interval)
    : LocalPlannerClassic(controller,pose_tracker,update_interval),
      LocalPlannerStar(controller,pose_tracker,update_interval)
{

}

double LocalPlannerStarN::f(double& g, double& score, double& heuristic){
    (void) score;
    return g + heuristic;
}
