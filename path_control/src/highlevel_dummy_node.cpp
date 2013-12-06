#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <path_msgs/NavigateToGoalAction.h>


// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const path_msgs::NavigateToGoalResultConstPtr& result)
{
    ROS_INFO("DONE [%d] with state %s", result->debug_test, state.toString().c_str());

//    ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const path_msgs::NavigateToGoalFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback [%d]", feedback->debug_test);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "highlevel_dummy");
    ros::NodeHandle nh;

    ROS_INFO("Let's go!");

    srand(ros::Time::now().toNSec());

    actionlib::SimpleActionClient<path_msgs::NavigateToGoalAction> client("navigate_to_goal", true);
    client.waitForServer();

    ROS_INFO("Client is set up");

    path_msgs::NavigateToGoalGoal goal;

    goal.goal_pose.header.stamp = ros::Time::now();
    goal.debug_test = rand()%100;
    goal.obstacle_mode = path_msgs::NavigateToGoalGoal::OBSTACLE_MODE_ABORT;

    ROS_INFO("Now send goal (%d)...", goal.debug_test);
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::Duration(2).sleep();

    goal.goal_pose.header.stamp = ros::Time::now();
    goal.debug_test = rand()%100;
    ROS_INFO("Now send another goal (%d)...", goal.debug_test);
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();
    return 0;
}

