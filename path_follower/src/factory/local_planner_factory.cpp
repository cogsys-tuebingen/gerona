/// HEADER
#include <path_follower/factory/local_planner_factory.h>

#include <path_follower/local_planner/local_planner_null.h>
#include <path_follower/local_planner/local_planner_transformer.h>
#include <path_follower/local_planner/local_planner_bfs_static.h>
#include <path_follower/local_planner/local_planner_bfs_reconf.h>
#include <path_follower/local_planner/local_planner_astar_n_static.h>
#include <path_follower/local_planner/local_planner_astar_g_static.h>
#include <path_follower/local_planner/local_planner_astar_n_reconf.h>
#include <path_follower/local_planner/local_planner_astar_g_reconf.h>
#include <path_follower/local_planner/local_planner_thetastar_n_static.h>
#include <path_follower/local_planner/local_planner_thetastar_g_static.h>
#include <path_follower/local_planner/local_planner_thetastar_n_reconf.h>
#include <path_follower/local_planner/local_planner_thetastar_g_reconf.h>

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
    if(opt_.c1()){
        local_planner->addConstraint(Dis2Path_Constraint::Ptr(new Dis2Path_Constraint));
    }

    if(opt_.c2()){
        local_planner->addConstraint(Dis2Obst_Constraint::Ptr(new Dis2Obst_Constraint));
    }

    if(opt_.s1() != 0.0){
        local_planner->addScorer(Dis2PathP_Scorer::Ptr(new Dis2PathP_Scorer), opt_.s1());
    }

    if(opt_.s2() != 0.0){
        local_planner->addScorer(Dis2PathD_Scorer::Ptr(new Dis2PathD_Scorer), opt_.s2());
    }

    if(opt_.s3() != 0.0){
        local_planner->addScorer(Curvature_Scorer::Ptr(new Curvature_Scorer), opt_.s3());
    }

    if(opt_.s4() != 0.0){
        local_planner->addScorer(CurvatureD_Scorer::Ptr(new CurvatureD_Scorer), opt_.s4());
    }

    if(opt_.s5() != 0.0){
        local_planner->addScorer(Level_Scorer::Ptr(new Level_Scorer), opt_.s5());
    }

    if(opt_.s6() != 0.0){
        local_planner->addScorer(Dis2Obst_Scorer::Ptr(new Dis2Obst_Scorer), opt_.s6());
    }

    return local_planner;
}

std::shared_ptr<LocalPlanner> LocalPlannerFactory::makeLocalPlanner(const std::string &name)
{
    std::string name_low = toLower(name);

    ROS_INFO("Use local planner algorithm '%s'", name.c_str());

    ROS_INFO("Maximum number of allowed nodes: %d", opt_.nnodes());
    ROS_INFO("Maximum tree depth: %d", opt_.depth());
    ROS_INFO("Update Interval: %.3f", opt_.uinterval());
    ROS_INFO("Maximal distance from path: %.3f", opt_.dis2p());
    ROS_INFO("Security distance around the robot: %.3f", opt_.adis());
    ROS_INFO("Security distance in front of the robot: %.3f", opt_.fdis());
    ROS_INFO("Steering angle: %.3f", opt_.s_angle());
    ROS_INFO("Intermediate Configurations: %d",opt_.ic());
    ROS_INFO("Intermediate Angles: %d",opt_.ia());
    ROS_INFO("Using current velocity: %s",opt_.use_v() ? "true" : "false");
    ROS_INFO("Length multiplying factor: %.3f",opt_.lmf());
    ROS_INFO("Coefficient of friction: %.3f",opt_.mu());
    ROS_INFO("Exponent factor: %.3f",opt_.ef());

    ROS_INFO("Constraint usage [%s, %s]", opt_.c1() ? "true" : "false",
             opt_.c2() ? "true" : "false");
    ROS_INFO("Scorer usage [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", opt_.s1(),opt_.s2(),
             opt_.s3(), opt_.s4(), opt_.s5(), opt_.s6());

    if(planner_loader_.isClassAvailable(name)) {
        ROS_INFO("Loading robot controller plugin '%s'", name.c_str());

        planner_constructors_[name_low] = [this, name]() -> std::shared_ptr<LocalPlanner> {
            return std::shared_ptr<LocalPlanner>(planner_loader_.createUnmanagedInstance(name));
        };

        return planner_constructors_[name_low]();
    }

    throw std::logic_error(std::string("Unknown local planner: ") + name + ". Shutdown.");
}
