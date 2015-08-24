#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <iostream>

std::ofstream file_stream;

unsigned int last_waypoint = 0;

unsigned int number_measurements = 0;
double d_sum = 0.,
	theta_e_sum = 0.,
	phi_sum = 0.,
	v_sum = 0.;

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
	if(!file_stream.is_open())
		return;

	std::vector<double> data = msg->data;
	unsigned int waypoint = (unsigned int) data[0];

	if (waypoint == last_waypoint) {
		d_sum += data[1];
		theta_e_sum += data[2];
		phi_sum += data[3];
		v_sum += data[4];
		++number_measurements;
//		ROS_INFO("number_measurements=%d", number_measurements);
		return;
	}


	file_stream << waypoint << "\t";
	file_stream << (d_sum / (double) number_measurements) << "\t";
	file_stream << (theta_e_sum / (double) number_measurements) << "\t";
	file_stream << (phi_sum / (double) number_measurements) << "\t";
	file_stream << (v_sum / (double) number_measurements) << "\t";

//	for (int i = 1; i < 5; ++i) {
//		file_stream << data[i] << "\t";
//	}

	file_stream << "\n";

	last_waypoint = waypoint;
	d_sum = theta_e_sum = phi_sum = v_sum = 0.;
	number_measurements = 0;
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

