/// HEADER
#include <path_follower/factory/follower_factory.h>

#include <path_follower/pathfollower.h>

#include <path_follower/factory/controller_factory.h>
#include <path_follower/factory/local_planner_factory.h>
#include <path_follower/factory/collision_avoider_factory.h>

#include <path_follower/controller/robotcontroller.h>
#include <path_follower/local_planner/abstract_local_planner.h>
#include <path_follower/collision_avoidance/collision_avoider.h>

#include <path_follower/utils/pose_tracker.h>

#include <ros/assert.h>

FollowerFactory::FollowerFactory(PathFollower &follower)
    : opt_(follower.getOptions()),
      follower_(follower),
      pose_tracker_(follower.getPoseTracker()),

      controller_factory_(new ControllerFactory(*PathFollowerParameters::getInstance())),
      local_planner_factory_(new LocalPlannerFactory(*LocalPlannerParameters::getInstance())),
      collision_avoider_factory_(new CollisionAvoiderFactory)
{

}

std::shared_ptr<PathFollowerConfig> FollowerFactory::construct(const PathFollowerConfigName &config)
{
    ROS_ASSERT_MSG(!config.controller.empty(), "No controller specified");
    ROS_ASSERT_MSG(!config.local_planner.empty(), "No local planner specified");
    //ROS_ASSERT_MSG(!config.collision_avoider.empty(), "No obstacle avoider specified");

    PathFollowerConfig result;
    result.controller_ = controller_factory_->makeController(config.controller);
    result.local_planner_ = local_planner_factory_->makeConstrainedLocalPlanner(config.local_planner);

    std::string collision_avoider = config.collision_avoider;
    if(collision_avoider.empty()) {
        collision_avoider = controller_factory_->getDefaultCollisionAvoider(config.controller);
        ROS_ASSERT_MSG(!collision_avoider.empty(), "No obstacle avoider specified");
    }
    result.collision_avoider_ = collision_avoider_factory_->makeObstacleAvoider(collision_avoider);

    ROS_ASSERT_MSG(result.controller_ != nullptr, "Controller was not set");
    ROS_ASSERT_MSG(result.local_planner_ != nullptr, "Local Planner was not set");
    ROS_ASSERT_MSG(result.collision_avoider_ != nullptr, "Obstacle Avoider was not set");

    // wiring
    result.collision_avoider_->setTransformListener(&pose_tracker_.getTransformListener());

    result.local_planner_->init(result.controller_.get(), &pose_tracker_);

    result.controller_->init(&pose_tracker_, result.collision_avoider_.get());

    pose_tracker_.setLocal(!result.local_planner_->isNull());

    return std::make_shared<PathFollowerConfig>(result);
}

void FollowerFactory::loadAll(std::vector<std::shared_ptr<RobotController> >& controllers)
{
    controller_factory_->loadAllControllers(controllers);
}
