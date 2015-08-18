#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <iostream>

std::ofstream file_stream;

unsigned int waypoint_counter = 0;

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
	if(!file_stream.is_open())
		return;

	std::vector<double> data = msg->data;
	unsigned int waypoint = (unsigned int) data[0];

	if (waypoint == waypoint_counter)
		return;

	waypoint_counter = waypoint;

	file_stream << waypoint << "\t";

	for (int i = 1; i < 5; ++i) {
		file_stream << data[i] << "\t";
	}

	file_stream << "\n";
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pose_goal_remapper");
	ros::NodeHandle n;

	auto s1 = n.subscribe("/test_output", 100, callback);

	std::string file_name = argc > 1 ? argv[1] : "output.csv";

	file_stream.open(file_name, std::ios::out);

	if(!file_stream.is_open()) {
		std::cerr << "Could not open file " << file_name << std::endl;
		return 0;
	}

	ROS_INFO("Opened file. Waiting for messages...");

	ros::Rate loop_rate(100);
	while (n.ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	file_stream.close();
	return 0;
}

