/// HEADER
#include <path_follower/local_planner/dis2obst_constraint.h>

Dis2Obst_Constraint::Dis2Obst_Constraint(ObstacleCloud::ConstPtr &msg, tf::Transformer &transformer):
    Constraint(),obstacles(msg),transformer_(transformer),now_(ros::Time::now())
{

}

Dis2Obst_Constraint::~Dis2Obst_Constraint()
{

}

bool Dis2Obst_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    if(!transformer_.waitForTransform("odom", "base_link", now_, ros::Duration(0.005))) {
        ROS_WARN_THROTTLE_NAMED(1, "local_path/Dis2Obst_Constraint", "cannot transform base_link to odom");
        sw.stop();
        return false;
    }

    tf::StampedTransform now_base_to_odom;
    transformer_.lookupTransform("odom", "base_link", now_, now_base_to_odom);

    tf::Point pt(point.x, point.y, point.orientation);

    tf::Transform transform_correction = now_base_to_odom.inverse();

    pt = transform_correction * pt;

    double closest_obst = std::numeric_limits<double>::infinity();

    ObstacleCloud::const_iterator point_it;
    for (point_it = obstacles->begin(); point_it != obstacles->end(); ++point_it){
        double x = (double)(point_it->x) - pt.x();
        double y = (double)(point_it->y) - pt.y();
        double a_diff = point.orientation - atan2(y,x);
        double dist = ((3-cos(a_diff)) * std::hypot(x, y))/2.0;
        if(dist < closest_obst) {
            closest_obst = dist;
            if(closest_obst <= 0.85){
                sw.stop();
                return false;
            }
        }
    }
    sw.stop();
    return true;
}
