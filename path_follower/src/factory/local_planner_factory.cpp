/// HEADER
#include <path_follower/factory/local_planner_factory.h>

#include <path_follower/local_planner/local_planner_null.h>

std::map<std::string, std::function<std::shared_ptr<LocalPlanner>()>> LocalPlannerFactory::planner_constructors_;

LocalPlannerFactory::LocalPlannerFactory(const LocalPlannerParameters &opt)
    : opt_(opt),

      planner_loader_("path_follower", "LocalPlanner")
{

}

std::shared_ptr<LocalPlanner> LocalPlannerFactory::makeConstrainedLocalPlanner(const std::string& name)
{
    std::shared_ptr<LocalPlanner> local_planner = makeLocalPlanner(name);

    //Begin Constraints and Scorers Construction
    if(opt_.use_distance_to_path_constraint()){
        local_planner->addConstraint(Dis2Path_Constraint::Ptr(new Dis2Path_Constraint));
    }

    if(opt_.use_distance_to_obstacle_constraint()){
        local_planner->addConstraint(Dis2Obst_Constraint::Ptr(new Dis2Obst_Constraint));
    }

    if(opt_.score_weight_distance_to_path() != 0.0){
        local_planner->addScorer(Dis2PathP_Scorer::Ptr(new Dis2PathP_Scorer),
                                 opt_.score_weight_distance_to_path());
    }

    if(opt_.score_weight_delta_distance_to_path() != 0.0){
        local_planner->addScorer(Dis2PathD_Scorer::Ptr(new Dis2PathD_Scorer),
                                 opt_.score_weight_delta_distance_to_path());
    }

    if(opt_.score_weight_curvature() != 0.0){
        local_planner->addScorer(Curvature_Scorer::Ptr(new Curvature_Scorer),
                                 opt_.score_weight_curvature());
    }

    if(opt_.score_weight_delta_curvature() != 0.0){
        local_planner->addScorer(CurvatureD_Scorer::Ptr(new CurvatureD_Scorer),
                                 opt_.score_weight_delta_curvature());
    }

    if(opt_.score_weight_level() != 0.0){
        local_planner->addScorer(Level_Scorer::Ptr(new Level_Scorer),
                                 opt_.score_weight_level());
    }

    if(opt_.score_weight_distance_to_obstacles() != 0.0){
        local_planner->addScorer(Dis2Obst_Scorer::Ptr(new Dis2Obst_Scorer),
                                 opt_.score_weight_distance_to_obstacles());
    }

    return local_planner;
}

std::shared_ptr<LocalPlanner> LocalPlannerFactory::makeLocalPlanner(const std::string &name)
{
    std::string name_low = toLower(name);

    ROS_INFO("Use local planner algorithm '%s'", name.c_str());

    ROS_INFO("Maximum number of allowed nodes: %d", opt_.max_num_nodes());
    ROS_INFO("Maximum tree depth: %d", opt_.max_depth());
    ROS_INFO("Update Interval: %.3f", opt_.update_interval());
    ROS_INFO("Maximal distance from path: %.3f", opt_.distance_to_path_constraint());
    ROS_INFO("Security distance around the robot: %.3f", opt_.safety_distance_surrounding());
    ROS_INFO("Security distance in front of the robot: %.3f", opt_.safety_distance_forward());
    ROS_INFO("Steering angle: %.3f", opt_.max_steering_angle());
    ROS_INFO("Intermediate Configurations: %d",opt_.curve_segment_subdivisions());
    ROS_INFO("Intermediate Angles: %d",opt_.intermediate_angles());
    ROS_INFO("Using current velocity: %s",opt_.use_velocity() ? "true" : "false");
    ROS_INFO("Length multiplying factor: %.3f",opt_.step_scale());
    ROS_INFO("Coefficient of friction: %.3f",opt_.mu());
    ROS_INFO("Exponent factor: %.3f",opt_.ef());

    ROS_INFO("Constraint usage [%s, %s]", opt_.use_distance_to_path_constraint() ? "true" : "false",
             opt_.use_distance_to_obstacle_constraint() ? "true" : "false");
    ROS_INFO("Scorer usage [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", opt_.score_weight_distance_to_path(),opt_.score_weight_delta_distance_to_path(),
             opt_.score_weight_curvature(), opt_.score_weight_delta_curvature(), opt_.score_weight_level(), opt_.score_weight_distance_to_obstacles());


    auto pos = planner_constructors_.find(name_low);
    if(pos != planner_constructors_.end()) {
        return pos->second();
    }

    if(planner_loader_.isClassAvailable(name)) {
        ROS_INFO("Loading robot controller plugin '%s'", name.c_str());

        planner_constructors_[name_low] = [this, name]() -> std::shared_ptr<LocalPlanner> {
            return std::shared_ptr<LocalPlanner>(planner_loader_.createUnmanagedInstance(name));
        };

        return planner_constructors_[name_low]();
    }

    throw std::logic_error(std::string("Unknown local planner: ") + name + ". Shutdown.");
}
