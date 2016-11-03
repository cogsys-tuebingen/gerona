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
#include <path_follower/controller/robotcontroller_kinematic_HBZ.h>

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


// SYSTEM
#include <stdexcept>

ControllerFactory::ControllerFactory(PathFollower &follower)
    : follower_(follower), opt_(follower.getOptions())
{

}

std::shared_ptr<RobotController> ControllerFactory::makeController(const std::string& name)
{
    ROS_INFO("Use robot controller '%s'", name.c_str());
    if (name == "ackermann_pid") {
        return std::make_shared<RobotController_Ackermann_Pid>(&follower_);

    } else if (name == "ackermann_purepursuit") {
        return std::make_shared<Robotcontroller_Ackermann_PurePursuit>(&follower_);

    } else if (name == "ackermann_inputscaling") {
        return std::make_shared<RobotController_Ackermann_Inputscaling>(&follower_);

    } else if (name == "ackermann_stanley") {
        return std::make_shared<RobotController_Ackermann_Stanley>(&follower_);

    } else if (name == "2steer_purepursuit") {
        return std::make_shared<RobotController_2Steer_PurePursuit>(&follower_);

    } else if (name == "2steer_stanley") {
        return std::make_shared<RobotController_2Steer_Stanley>(&follower_);

    } else if (name == "2steer_inputscaling") {
        return std::make_shared<RobotController_2Steer_InputScaling>(&follower_);

    } else if (name == "unicycle_inputscaling") {
        return std::make_shared<RobotController_Unicycle_InputScaling>(&follower_);

    } else if (name == "patsy_pid") {
        std::shared_ptr<RobotControllerTrailer> ctrl(new RobotControllerTrailer(&follower_,&follower_.getNodeHandle()));
        return ctrl;

    } else if (name == "omnidrive_orthexp") {
        return std::make_shared<RobotController_Omnidrive_OrthogonalExponential>(&follower_);

    } else if (name == "ackermann_orthexp") {
        return std::make_shared<RobotController_Ackermann_OrthogonalExponential>(&follower_);

    } else if (name == "differential_orthexp") {
        return std::make_shared<RobotController_Differential_OrthogonalExponential>(&follower_);

    } else if (name == "kinematic_SLP") {
        return std::make_shared<RobotController_Kinematic_SLP>(&follower_);

    } else if (name == "dynamic_SLP") {
        return std::make_shared<RobotController_Dynamic_SLP>(&follower_);

    } else if (name == "kinematic_HBZ") {
        return std::make_shared<RobotController_Kinematic_HBZ>(&follower_);

    } else if (name == "ICR_CCW") {
        return std::make_shared<RobotController_ICR_CCW>(&follower_);

    } else {
        throw std::logic_error("Unknown robot controller. Shutdown.");
    }
}

std::shared_ptr<LocalPlanner>
ControllerFactory::makeLocalPlanner(const std::string& name,
                                    tf::TransformListener& pose_listener,
                                    const ros::Duration& uinterval)
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


    std::shared_ptr<LocalPlanner> planner;
    if(name == "AStar"){
        planner = std::make_shared<LocalPlannerAStarNStatic>(*&follower_, pose_listener, uinterval);
    }else if(name == "AStarG"){
        planner = std::make_shared<LocalPlannerAStarGStatic>(*&follower_, pose_listener, uinterval);
    }else if(name == "ThetaStar"){
        planner = std::make_shared<LocalPlannerThetaStarNStatic>(*&follower_, pose_listener, uinterval);
    }else if(name == "ThetaStarG"){
        planner = std::make_shared<LocalPlannerThetaStarGStatic>(*&follower_, pose_listener, uinterval);
    }else if(name == "AStarR"){
        planner = std::make_shared<LocalPlannerAStarNReconf>(*&follower_, pose_listener, uinterval);
    }else if(name == "AStarGR"){
        planner = std::make_shared<LocalPlannerAStarGReconf>(*&follower_, pose_listener, uinterval);
    }else if(name == "ThetaStarR"){
        planner = std::make_shared<LocalPlannerThetaStarNReconf>(*&follower_, pose_listener, uinterval);
    }else if(name == "ThetaStarGR"){
        planner = std::make_shared<LocalPlannerThetaStarGReconf>(*&follower_, pose_listener, uinterval);
    }else if(name == "BFS"){
        planner = std::make_shared<LocalPlannerBFSStatic>(*&follower_, pose_listener, uinterval);
    }else if(name == "BFSR"){
        planner = std::make_shared<LocalPlannerBFSReconf>(*&follower_, pose_listener, uinterval);
    }else if(name == "Transformer"){
        planner = std::make_shared<LocalPlannerTransformer>(*&follower_, pose_listener, uinterval);
    }else if(name == "NULL"){
        planner = std::make_shared<LocalPlannerNull>(*&follower_, pose_listener);
    }else {
        throw std::logic_error("Unknown local planner algorithm. Shutdown.");
    }


    planner->setParams(opt_.nnodes(), opt_.ic(), opt_.dis2p(), opt_.adis(),
                       opt_.fdis(),opt_.s_angle(), opt_.ia(), opt_.lmf(),
                       opt_.depth(), opt_.mu(), opt_.ef());

    return planner;
}

std::shared_ptr<ObstacleAvoider>
ControllerFactory::makeObstacleAvoider(const std::string& name,
                                       tf::TransformListener& pose_listener,
                                       std::shared_ptr<RobotController>& controller)
{
    ROS_INFO("Use robot controller '%s'", name.c_str());
    if (name == "ackermann_pid") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "ackermann_purepursuit") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "ackermann_inputscaling") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "ackermann_stanley") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "2steer_purepursuit") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "2steer_stanley") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "2steer_inputscaling") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "unicycle_inputscaling") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "patsy_pid") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorPatsy>(&pose_listener,
                                                           dynamic_cast<RobotControllerTrailer*>(controller.get()));

    } else if (name == "omnidrive_orthexp") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorOmnidrive>(&pose_listener);

    } else if (name == "ackermann_orthexp") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorOmnidrive>(&pose_listener);

    } else if (name == "differential_orthexp") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorOmnidrive>(&pose_listener);

    } else if (name == "kinematic_SLP") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "dynamic_SLP") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "kinematic_HBZ") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else if (name == "ICR_CCW") {
        if (opt_.obstacle_avoider_use_collision_box())
            return std::make_shared<ObstacleDetectorAckermann>(&pose_listener);

    } else {
        throw std::logic_error("Unknown robot controller. Shutdown.");
    }


    //  if no obstacle avoider was set, use the none-avoider
    return std::make_shared<NoneAvoider>();
}
