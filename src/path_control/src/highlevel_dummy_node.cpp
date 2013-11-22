#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <path_msgs/NavigateToGoalAction.h>


// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const path_msgs::NavigateToGoalResultConstPtr& result)
{
    ROS_INFO("DONE");

    ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const path_msgs::NavigateToGoalFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "highlevel_dummy");
    ros::NodeHandle nh;

    ROS_INFO("Let's go!");

    actionlib::SimpleActionClient<path_msgs::NavigateToGoalAction> client("navigate_to_goal", true);
    client.waitForServer();

    ROS_INFO("Client is set up");

    path_msgs::NavigateToGoalGoal goal;

    goal.goal_pose.header.stamp = ros::Time::now();
    goal.goal_pose.pose.position.x = 42;
    goal.obstacle_mode = path_msgs::NavigateToGoalGoal::OBSTACLE_MODE_ABORT;

    ROS_INFO("Now send goal...");
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();
    return 0;
}

