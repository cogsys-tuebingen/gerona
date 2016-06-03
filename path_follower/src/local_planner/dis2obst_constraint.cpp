/// HEADER
#include <path_follower/local_planner/dis2obst_constraint.h>

Dis2Obst_Constraint::Dis2Obst_Constraint(ObstacleCloud::ConstPtr &msg, tf::Transformer &transformer):
    obstacles(msg),transformer_(transformer),now_(ros::Time::now())
{

}

Dis2Obst_Constraint::~Dis2Obst_Constraint()
{

}

bool Dis2Obst_Constraint::isSatisfied(const tf::Point& point){
    if(!transformer_.waitForTransform("odom", "base_link", now_, ros::Duration(0.1))) {
        ROS_WARN_THROTTLE_NAMED(1, "local_path/Dis2Obst_Constraint", "cannot transform base_link to odom");
        return std::numeric_limits<double>::infinity();
    }

    tf::StampedTransform now_base_to_odom;
    transformer_.lookupTransform("odom", "base_link", now_, now_base_to_odom);

    tf::Transform transform_correction = now_base_to_odom.inverse();

    tf::Point pt = transform_correction * point;

    double closest_obst = std::numeric_limits<double>::infinity();

    ObstacleCloud::const_iterator point_it;
    for (point_it = obstacles->begin(); point_it != obstacles->end(); ++point_it){
        double x = pt.x() - (double)(point_it->x);
        double y = pt.y() - (double)(point_it->y);
        double dist = std::hypot(x, y);
        if(dist < closest_obst) {
            closest_obst = dist;
            if(closest_obst <= 0.85){
                return false;
            }
        }
    }

    return true;
}
