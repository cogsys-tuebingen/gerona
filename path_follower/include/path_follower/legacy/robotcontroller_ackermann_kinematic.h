#ifndef ROBOTCONTROLLER_ACKERMANN_KINEMATIC_H
#define ROBOTCONTROLLER_ACKERMANN_KINEMATIC_H

#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>

#include <visualization_msgs/Marker.h>

#include <ros/ros.h>

class RobotController_Ackermann_Kinematic : public RobotController_Interpolation
{
public:
	RobotController_Ackermann_Kinematic(PathFollower* _path_follower);
	virtual ~RobotController_Ackermann_Kinematic(){}

	virtual void stopMotion();
	virtual void start();
	virtual bool isOmnidirectional() const {
		return true;
	}

protected:
	virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
	virtual void publishMoveCommand(const MoveCommand &cmd) const;

private:
	struct ControllerParameters : public RobotController_Interpolation::InterpolationParameters {
		P<double> vehicle_length;
		P<double> k;

		ControllerParameters() :
			vehicle_length(this, "~vehicle_length", 0.3, "axis-centre distance"),
			k(this, "~k", 0.5, "Tuning factor")
		{}

	} params;

	const RobotController_Interpolation::InterpolationParameters& getParameters() const
	{
		return params;
	}

	void reset();

	bool reachedGoal(const Eigen::Vector3d& pose) const;

	//double computeErrorRearAxis(const Eigen::Vector3d& pose, Eigen::Vector3d& wayPoint) const;
	//double computeErrorTheta() const;
	void computeErrors(const Eigen::Vector3d& pose, double& errorRearAxis,double& errorTheta) const;
	double computeAlpha1(const double x2, const double errorRearAxis,
								const double curvature, const double tanErrorTheta) const;

	ros::NodeHandle node_handle;
	ros::Publisher path_interpol_pub;

	MoveCommand move_cmd;

	double k1, k2, k3;
	double delta;
	ros::Time oldTime;
};

#endif // ROBOTCONTROLLER_ACKERMANN_KINEMATIC_H
