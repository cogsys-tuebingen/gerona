#include <ros/ros.h>
//#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::PoseStamped curPose;
ros::Publisher pub;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr msg) {
	 curPose = *msg;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& _) {
	pub.publish(curPose);
}

int main(int argc, char **argv) {
	 ros::init(argc, argv, "pose_goal_remapper");
    
    ros::NodeHandle n;
    
	 pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    
	 n.subscribe("slam_out_pose", 1, poseCallback);

	 n.subscribe("move_base_simple/goal", 1, goalCallback);
    
    ros::Rate loop_rate(10);
    while (n.ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

