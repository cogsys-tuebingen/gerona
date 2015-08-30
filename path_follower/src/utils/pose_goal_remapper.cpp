#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped curPose;
ros::Publisher pub;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	curPose = *msg;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	ROS_INFO("Republishing goal...");
	pub.publish(curPose);
	ROS_INFO_STREAM("Old goal: " << (*msg) << ", new goal: " << curPose);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pose_goal_remapper");

	ros::NodeHandle n;

	pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

	auto s1 = n.subscribe("/slam_out_pose", 10, callback);

	auto s2 = n.subscribe("/rviz_goal", 1, goalCallback);

	ROS_INFO("Started remapping. Waiting for goals...");

	ros::Rate loop_rate(10);
	while (n.ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

