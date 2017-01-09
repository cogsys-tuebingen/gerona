/// HEADER
#include <path_follower/factory/controller_factory.h>

#include <path_follower/pathfollower.h>

// Controller/Models
#include <path_follower/controller/robotcontroller_ackermann_pid.h>
#include <path_follower/controller/robotcontrollertrailer.h>
#include <path_follower/controller/robotcontroller_ICR_CCW.h>
#include <path_follower/controller/robotcontroller_ackermann_orthexp.h>
#include <path_follower/controller/robotcontroller_ackermann_purepursuit.h>
#include <path_follower/controller/robotcontroller_ackermann_inputscaling.h>
#include <path_follower/controller/robotcontroller_ackermann_stanley.h>
#include <path_follower/controller/robotcontroller_2steer_purepursuit.h>
#include <path_follower/controller/robotcontroller_2steer_stanley.h>
#include <path_follower/controller/robotcontroller_2steer_inputscaling.h>
#include <path_follower/controller/robotcontroller_unicycle_inputscaling.h>
#include <path_follower/controller/robotcontroller_omnidrive_orthexp.h>
#include <path_follower/controller/robotcontroller_differential_orthexp.h>
#include <path_follower/controller/robotcontroller_kinematic_SLP.h>
#include <path_follower/controller/robotcontroller_dynamic_SLP.h>

#include <path_follower/obstacle_avoidance/noneavoider.hpp>
#include <path_follower/obstacle_avoidance/obstacledetectorackermann.h>
#include <path_follower/obstacle_avoidance/obstacledetectoromnidrive.h>
#include <path_follower/obstacle_avoidance/obstacledetectorpatsy.h>

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

ControllerFactory::ControllerFactory(PathFollower &follower)
    : follower_(follower), opt_(follower.getOptions()), pose_tracker_(follower.getPoseTracker()),

      controller_loader("path_follower", "RobotController")
{

}

std::shared_ptr<PathFollowerConfig> ControllerFactory::construct(const std::string& config)
{
    PathFollowerConfig result;

    if(config.empty()) {
        result.controller_ = makeController(opt_.controller());
        result.local_planner_ = makeConstrainedLocalPlanner(opt_.algo());
        result.obstacle_avoider_ = makeObstacleAvoider(opt_.controller());

    } else {
        if(config == "HBZ_forward_backward") {
            result.controller_ = makeController("RobotController_Kinematic_HBZ");
            result.local_planner_ = makeConstrainedLocalPlanner("NULL");
            result.obstacle_avoider_ = makeObstacleAvoider("RobotController_Kinematic_HBZ");

        } else {            
            result.controller_ = makeController(config);
            result.local_planner_ = makeConstrainedLocalPlanner("NULL");
            result.obstacle_avoider_ = makeObstacleAvoider(config);
        }
    }

    ROS_ASSERT_MSG(result.controller_ != nullptr, "Controller was not set");
    ROS_ASSERT_MSG(result.local_planner_ != nullptr, "Local Planner was not set");
    ROS_ASSERT_MSG(result.obstacle_avoider_ != nullptr, "Obstacle Avoider was not set");

    // wiring
    result.obstacle_avoider_->setTransformListener(&pose_tracker_.getTransformListener());

    ros::Duration uinterval(opt_.uinterval());
    result.local_planner_->init(result.controller_.get(), &pose_tracker_, uinterval);

    result.controller_->init(&pose_tracker_, result.obstacle_avoider_.get(), &opt_);

    pose_tracker_.setLocal(!result.local_planner_->isNull());

    result.local_planner_->setParams(opt_.nnodes(), opt_.ic(), opt_.dis2p(), opt_.adis(),
                                     opt_.fdis(),opt_.s_angle(), opt_.ia(), opt_.lmf(),
                                     opt_.depth(), opt_.mu(), opt_.ef());

    return std::make_shared<PathFollowerConfig>(result);
}


std::shared_ptr<RobotController> ControllerFactory::makeController(const std::string& name)
{
    std::string name_low = name;
    std::transform(name_low.begin(), name_low.end(), name_low.begin(), ::tolower);

    std::vector<std::string> classes = controller_loader.getDeclaredClasses();
    std::vector<std::string> classes_low;
    classes_low.reserve(classes.size());
    for(const std::string& s : classes) {
        std::string low = s;
        std::transform(low.begin(), low.end(), low.begin(), ::tolower);
        classes_low.push_back(low);
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
        ROS_INFO("Use robot controller plugin '%s'", plugin_class.c_str());

        return std::shared_ptr<RobotController>(controller_loader.createUnmanagedInstance(plugin_class));
    }

    // legacy controllers:
    // TODO: move to plugins
    ROS_INFO("Use legacy robot controller '%s'", name.c_str());

    if (name == "ackermann_pid") {
        return std::make_shared<RobotController_Ackermann_Pid>();

    } else if (name == "ackermann_purepursuit") {
        return std::make_shared<Robotcontroller_Ackermann_PurePursuit>();

    } else if (name == "ackermann_inputscaling") {
        return std::make_shared<RobotController_Ackermann_Inputscaling>();

    } else if (name == "ackermann_stanley") {
        return std::make_shared<RobotController_Ackermann_Stanley>();

    } else if (name == "2steer_purepursuit") {
        return std::make_shared<RobotController_2Steer_PurePursuit>();

    } else if (name == "2steer_stanley") {
        return std::make_shared<RobotController_2Steer_Stanley>();

    } else if (name == "2steer_inputscaling") {
        return std::make_shared<RobotController_2Steer_InputScaling>();

    } else if (name == "unicycle_inputscaling") {
        return std::make_shared<RobotController_Unicycle_InputScaling>();

    } else if (name == "patsy_pid") {
        return std::make_shared<RobotControllerTrailer>();

    } else if (name == "omnidrive_orthexp") {
        return std::make_shared<RobotController_Omnidrive_OrthogonalExponential>();

    } else if (name == "ackermann_orthexp") {
        return std::make_shared<RobotController_Ackermann_OrthogonalExponential>();

    } else if (name == "differential_orthexp") {
        return std::make_shared<RobotController_Differential_OrthogonalExponential>();

    } else if (name == "kinematic_SLP") {
        return std::make_shared<RobotController_Kinematic_SLP>();

    } else if (name == "dynamic_SLP") {
        return std::make_shared<RobotController_Dynamic_SLP>();

    } else if (name == "ICR_CCW") {
        return std::make_shared<RobotController_ICR_CCW>();

    } else {
        throw std::logic_error(std::string("Unknown robot controller: ") + name + ". Shutdown.");
    }
}

std::shared_ptr<LocalPlanner> ControllerFactory::makeConstrainedLocalPlanner(const std::string& name)
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

std::shared_ptr<LocalPlanner> ControllerFactory::makeLocalPlanner(const std::string &name)
{
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

std::shared_ptr<ObstacleAvoider> ControllerFactory::makeObstacleAvoider(const std::string &name)
{
    if (name == "ackermann_pid") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "ackermann_purepursuit") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "ackermann_inputscaling") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "ackermann_stanley") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "2steer_purepursuit") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "2steer_stanley") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "2steer_inputscaling") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "unicycle_inputscaling") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "patsy_pid") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorPatsy>();

    } else if (name == "omnidrive_orthexp") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorOmnidrive>();

    } else if (name == "ackermann_orthexp") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorOmnidrive>();

    } else if (name == "differential_orthexp") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorOmnidrive>();

    } else if (name == "kinematic_SLP") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "dynamic_SLP") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else if (name == "ICR_CCW") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>();

    } else {
        if (opt_.obstacle_avoider_use_collision_box()) {
            // TODO: think about how to implement this for plug-ins
            ROS_WARN_STREAM("Unknown robot controller: " << name << ". Defaulting to AckermannDetector.");
            return std::make_shared<ObstacleDetectorAckermann>();
        }
    }
    //  if no obstacle avoider was set, use the none-avoider
    return std::make_shared<NoneAvoider>();
}
