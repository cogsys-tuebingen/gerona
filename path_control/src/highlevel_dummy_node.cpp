#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <path_msgs/NavigateToGoalAction.h>
#include <nav_msgs/OccupancyGrid.h>

/**
 * @brief Simple high level control dummy for testing.
 *
 * This dummy subscribes for the goals set manually in rviz. The goals are then wrapped in an action and send to
 * path_control.
 */

class HighDummy
{
public:
    HighDummy(ros::NodeHandle &nh):
        nh_(nh),
        client_("navigate_to_goal", true)
    {
        srand(ros::Time::now().toNSec());
        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/rviz_goal", 0, &HighDummy::goalCb, this);
        client_.waitForServer();

        ROS_INFO("Client is set up");
    }


private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<path_msgs::NavigateToGoalAction> client_;
    ros::Subscriber goal_sub_;


    // Called once when the goal completes
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const path_msgs::NavigateToGoalResultConstPtr& result)
    {
        ROS_INFO("DONE [%d] with state %s", result->debug_test, state.toString().c_str());
        ROS_INFO("Additional Text: %s", state.getText().c_str());
    }

    // Called once when the goal becomes active
    void activeCb()
    {
        ROS_INFO("Goal just went active");
    }

    // Called every time feedback is received for the goal
    void feedbackCb(const path_msgs::NavigateToGoalFeedbackConstPtr& feedback)
    {
        //ROS_INFO("Got Feedback [%d]", feedback->debug_test);
    }

    void goalCb(const geometry_msgs::PoseStampedConstPtr &pose)
    {
        ROS_INFO("Send goal...");

        path_msgs::NavigateToGoalGoal goal;
        goal.goal_pose = *pose;
        goal.obstacle_mode = path_msgs::NavigateToGoalGoal::OBSTACLE_MODE_ABORT;
        goal.debug_test = rand()%100;

        client_.sendGoal(goal,
                         boost::bind(&HighDummy::doneCb, this, _1, _2),
                         boost::bind(&HighDummy::activeCb, this),
                         boost::bind(&HighDummy::feedbackCb, this, _1));
    }


};


int main(int argc, char** argv) {
    ros::init(argc, argv, "highlevel_dummy");
    ros::NodeHandle nh;

    HighDummy dummy(nh);

    ros::spin();
    return 0;
}

