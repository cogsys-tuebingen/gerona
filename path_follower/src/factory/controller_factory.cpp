/// HEADER
#include <path_follower/factory/controller_factory.h>

#include <path_follower/pathfollower.h>

// Controller/Models
#include <path_follower/controller/robotcontroller.h>

#include <path_follower/collision_avoidance/none_avoider.hpp>
#include <path_follower/collision_avoidance/collision_detector_ackermann.h>
#include <path_follower/collision_avoidance/collision_detector_omnidrive.h>

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

#include <path_follower/utils/pose_tracker.h>

// SYSTEM
#include <stdexcept>

std::map<std::string, std::function<std::shared_ptr<RobotController>()>> ControllerFactory::controller_constructors_;
std::map<std::string, std::string> ControllerFactory::default_collision_detectors_;

ControllerFactory::ControllerFactory(PathFollower &follower)
    : follower_(follower), opt_(follower.getOptions()), pose_tracker_(follower.getPoseTracker()),

      controller_loader("path_follower", "RobotController")
{

}

std::shared_ptr<PathFollowerConfig> ControllerFactory::construct(const PathFollowerConfigName &config)
{
    ROS_ASSERT_MSG(!config.controller.empty(), "No controller specified");
    ROS_ASSERT_MSG(!config.local_planner.empty(), "No local planner specified");
    //ROS_ASSERT_MSG(!config.collision_avoider.empty(), "No obstacle avoider specified");

    PathFollowerConfig result;
    result.controller_ = makeController(config.controller);
    result.local_planner_ = makeConstrainedLocalPlanner(config.local_planner);

    std::string collision_avoider = config.collision_avoider;
    if(collision_avoider.empty()) {
        auto key = default_collision_detectors_.find(config.controller);
        if(key != default_collision_detectors_.end()) {
            collision_avoider = key->second;
        }

        ROS_ASSERT_MSG(!collision_avoider.empty(), "No obstacle avoider specified");
    }
    result.collision_avoider_ = makeObstacleAvoider(collision_avoider);

    ROS_ASSERT_MSG(result.controller_ != nullptr, "Controller was not set");
    ROS_ASSERT_MSG(result.local_planner_ != nullptr, "Local Planner was not set");
    ROS_ASSERT_MSG(result.collision_avoider_ != nullptr, "Obstacle Avoider was not set");

    // wiring
    result.collision_avoider_->setTransformListener(&pose_tracker_.getTransformListener());

    ros::Duration uinterval(opt_.local_planner.uinterval());
    result.local_planner_->init(result.controller_.get(), &pose_tracker_, uinterval);

    result.controller_->init(&pose_tracker_, result.collision_avoider_.get(), &opt_);

    pose_tracker_.setLocal(!result.local_planner_->isNull());

    LocalPlannerParameters local = opt_.local_planner;
    result.local_planner_->setParams(local.nnodes(), local.ic(), local.dis2p(), local.adis(),
                                     local.fdis(),local.s_angle(), local.ia(), local.lmf(),
                                     local.depth(), local.mu(), local.ef());

    ROS_WARN_STREAM("nnodes: " << local.nnodes());

    return std::make_shared<PathFollowerConfig>(result);
}

std::string ControllerFactory::toLower(const std::string& s)
{
    std::string s_low = s;
    std::transform(s_low.begin(), s_low.end(), s_low.begin(), ::tolower);
    return s_low;
}

std::shared_ptr<RobotController> ControllerFactory::makeController(const std::string& name)
{
    std::string name_low = toLower(name);

    auto pos = controller_constructors_.find(name_low);
    if(pos != controller_constructors_.end()) {
        return pos->second();
    }

    std::vector<std::string> classes = controller_loader.getDeclaredClasses();
    std::vector<std::string> classes_low;
    classes_low.reserve(classes.size());
    for(const std::string& s : classes) {
        classes_low.push_back(toLower(s));
    }

    std::string plugin_class;

    for(int i = 0, n = classes.size(); i < n; ++i) {
        const std::string& available = classes_low.at(i);

        // first try direct name
        if(available == name_low) {
            plugin_class = classes.at(i);
            break;
        }

        std::string conventional = std::string("robotcontroller_") + name_low;

        // then try the conventional naming scheme
        if(conventional == available) {
            plugin_class = classes.at(i);
            break;
        }
    }

    if(!plugin_class.empty() && controller_loader.isClassAvailable(plugin_class)) {
        ROS_INFO("Loading robot controller plugin '%s'", plugin_class.c_str());

        controller_constructors_[name_low] = [this, plugin_class]() -> std::shared_ptr<RobotController> {
            return std::shared_ptr<RobotController>(controller_loader.createUnmanagedInstance(plugin_class));
        };

        return controller_constructors_[name_low]();
    }

    throw std::logic_error(std::string("Unknown robot controller: ") + name + ". Shutdown.");
}

std::shared_ptr<LocalPlanner> ControllerFactory::makeConstrainedLocalPlanner(const std::string& name)
{
    std::shared_ptr<LocalPlanner> local_planner = makeLocalPlanner(name);

    //Begin Constraints and Scorers Construction
    if(opt_.local_planner.c1()){
        local_planner->addConstraint(Dis2Path_Constraint::Ptr(new Dis2Path_Constraint));
    }

    if(opt_.local_planner.c2()){
        local_planner->addConstraint(Dis2Obst_Constraint::Ptr(new Dis2Obst_Constraint));
    }

    if(opt_.local_planner.s1() != 0.0){
        local_planner->addScorer(Dis2PathP_Scorer::Ptr(new Dis2PathP_Scorer), opt_.local_planner.s1());
    }

    if(opt_.local_planner.s2() != 0.0){
        local_planner->addScorer(Dis2PathD_Scorer::Ptr(new Dis2PathD_Scorer), opt_.local_planner.s2());
    }

    if(opt_.local_planner.s3() != 0.0){
        local_planner->addScorer(Curvature_Scorer::Ptr(new Curvature_Scorer), opt_.local_planner.s3());
    }

    if(opt_.local_planner.s4() != 0.0){
        local_planner->addScorer(CurvatureD_Scorer::Ptr(new CurvatureD_Scorer), opt_.local_planner.s4());
    }

    if(opt_.local_planner.s5() != 0.0){
        local_planner->addScorer(Level_Scorer::Ptr(new Level_Scorer), opt_.local_planner.s5());
    }

    if(opt_.local_planner.s6() != 0.0){
        local_planner->addScorer(Dis2Obst_Scorer::Ptr(new Dis2Obst_Scorer), opt_.local_planner.s6());
    }

    return local_planner;
}

std::shared_ptr<LocalPlanner> ControllerFactory::makeLocalPlanner(const std::string &name)
{
    ROS_INFO("Use local planner algorithm '%s'", name.c_str());

    ROS_INFO("Maximum number of allowed nodes: %d", opt_.local_planner.nnodes());
    ROS_INFO("Maximum tree depth: %d", opt_.local_planner.depth());
    ROS_INFO("Update Interval: %.3f", opt_.local_planner.uinterval());
    ROS_INFO("Maximal distance from path: %.3f", opt_.local_planner.dis2p());
    ROS_INFO("Security distance around the robot: %.3f", opt_.local_planner.adis());
    ROS_INFO("Security distance in front of the robot: %.3f", opt_.local_planner.fdis());
    ROS_INFO("Steering angle: %.3f", opt_.local_planner.s_angle());
    ROS_INFO("Intermediate Configurations: %d",opt_.local_planner.ic());
    ROS_INFO("Intermediate Angles: %d",opt_.local_planner.ia());
    ROS_INFO("Using current velocity: %s",opt_.local_planner.use_v() ? "true" : "false");
    ROS_INFO("Length multiplying factor: %.3f",opt_.local_planner.lmf());
    ROS_INFO("Coefficient of friction: %.3f",opt_.local_planner.mu());
    ROS_INFO("Exponent factor: %.3f",opt_.local_planner.ef());

    ROS_INFO("Constraint usage [%s, %s]", opt_.local_planner.c1() ? "true" : "false",
             opt_.local_planner.c2() ? "true" : "false");
    ROS_INFO("Scorer usage [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", opt_.local_planner.s1(),opt_.local_planner.s2(),
             opt_.local_planner.s3(), opt_.local_planner.s4(), opt_.local_planner.s5(), opt_.local_planner.s6());


    if(name == "AStar"){
        return std::make_shared<LocalPlannerAStarNStatic>();
    } else if(name == "AStarG"){
        return std::make_shared<LocalPlannerAStarGStatic>();
    } else if(name == "ThetaStar"){
        return std::make_shared<LocalPlannerThetaStarNStatic>();
    } else if(name == "ThetaStarG"){
        return std::make_shared<LocalPlannerThetaStarGStatic>();
    } else if(name == "AStarR"){
        return std::make_shared<LocalPlannerAStarNReconf>();
    } else if(name == "AStarGR"){
        return std::make_shared<LocalPlannerAStarGReconf>();
    } else if(name == "ThetaStarR"){
        return std::make_shared<LocalPlannerThetaStarNReconf>();
    } else if(name == "ThetaStarGR"){
        return std::make_shared<LocalPlannerThetaStarGReconf>();
    } else if(name == "BFS"){
        return std::make_shared<LocalPlannerBFSStatic>();
    } else if(name == "BFSR"){
        return std::make_shared<LocalPlannerBFSReconf>();
    } else if(name == "Transformer"){
        return std::make_shared<LocalPlannerTransformer>();
    } else if(name == "NULL"){
        return std::make_shared<LocalPlannerNull>();
    }else {
        throw std::logic_error("Unknown local planner algorithm. Shutdown.");
    }
}

std::shared_ptr<CollisionAvoider> ControllerFactory::makeObstacleAvoider(const std::string &name)
{
    if(name == "default_collision_avoider") {
        return std::make_shared<CollisionDetectorOmnidrive>();
    } else {
        if (name == "omnidive") {
            return std::make_shared<CollisionDetectorOmnidrive>();
        } else if (name == "ackermann") {
            return std::make_shared<CollisionDetectorAckermann>();
        }
    }
    ROS_WARN_STREAM("No collision_avoider defined with the name '" << name << "'. Defaulting to Omnidrive.");
    return std::make_shared<CollisionDetectorOmnidrive>();
}
