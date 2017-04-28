// HEADER
#include <path_follower/controller/robotcontroller_potential_field.h>

// THIRD PARTY
#include <tf/tf.h>
#include <nav_msgs/Path.h>

// PROJECT
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <path_follower/pathfollowerparameters.h>
#include <path_follower/obstacle_avoidance/obstacleavoider.h>
#include <cslibs_utils/MathHelper.h>

// SYSTEM
#include <boost/algorithm/clamp.hpp>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Potential_Field, potential_field, default_collision_avoider);

RobotController_Potential_Field::RobotController_Potential_Field():
    RobotController(),
    cmd_(this),
    FAttX(0.0),
    FAttY(0.0),
    FResX(0.0),
    FResY(0.0),
    vn_(0.0),
    proj_ind_(0),
    xe_(0.0),
    ye_(0.0),
    theta_e_(0.0),
    mGoalPosX(0.0),
    mGoalPosY(0.0)
{
    F_pub = nh_.advertise<visualization_msgs::MarkerArray>("potential_field_vectors", 0);
}


void RobotController_Potential_Field::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Potential_Field::initialize()
{
    RobotController::initialize();

    // initialize the markers for the visualization
    initializeMarkers();

    // desired velocity
    vn_ = std::min(global_opt_->max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);
}

void RobotController_Potential_Field::reset()
{
    RobotController::reset();
}


void RobotController_Potential_Field::setPath(Path::Ptr path)
{
    RobotController::setPath(path);
}


/**
 * Initializes all the markers used for the visualization of FAtt, FRep and FRes.
 */
void RobotController_Potential_Field::initializeMarkers()
{
    //force markers should last 1.5 seconds
    //ros::Duration dur(1.5);

    // FAtt marker
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "F_att";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0f;
    marker.pose.position.y = 0.0f;
    marker.pose.position.z = 0.0f;
    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 1.0f;
    marker.pose.orientation.w = 0.0f;
    marker.scale.x = 0.01f;
    marker.scale.y = 0.1f;
    marker.scale.z = 0.1f;
    marker.color.a = 1.0f;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    //marker.lifetime = dur;

    FMarkers.markers.push_back(marker);

    // FRep marker
    FrepMarker.header.frame_id = "base_link";
    FrepMarker.header.stamp = ros::Time();
    FrepMarker.ns = "F_rep";
    FrepMarker.id = 37;
    FrepMarker.type = visualization_msgs::Marker::ARROW;
    FrepMarker.action = visualization_msgs::Marker::ADD;
    FrepMarker.pose.position.x = 0.0f;
    FrepMarker.pose.position.y = 0.0f;
    FrepMarker.pose.position.z = 0.0f;
    FrepMarker.pose.orientation.x = 0.0f;
    FrepMarker.pose.orientation.y = 0.0f;
    FrepMarker.pose.orientation.z = 1.0f;
    FrepMarker.pose.orientation.w = 0.0f;
    FrepMarker.scale.x = 0.01f;
    FrepMarker.scale.y = 0.01f;
    FrepMarker.scale.z = 0.01f;
    FrepMarker.color.a = 1.0f;
    FrepMarker.color.r = 1.0f;
    FrepMarker.color.g = 0.0f;
    FrepMarker.color.b = 0.0f;
    //markers[i].lifetime = dur;
    FMarkers.markers.push_back(FrepMarker);

    // FRes marker
    FResMarker.header.frame_id = "base_link";
    FResMarker.header.stamp = ros::Time();
    FResMarker.ns = "F_res";
    FResMarker.id = 0;
    FResMarker.type = visualization_msgs::Marker::ARROW;
    FResMarker.action = visualization_msgs::Marker::ADD;
    FResMarker.pose.position.x = 0.0f;
    FResMarker.pose.position.y = 0.0f;
    FResMarker.pose.position.z = 0.0f;
    FResMarker.pose.orientation.x = 0.0f;
    FResMarker.pose.orientation.y = 0.0f;
    FResMarker.pose.orientation.z = 1.0f;
    FResMarker.pose.orientation.w = 0.0f;
    FResMarker.scale.x = 0.1f;
    FResMarker.scale.y = 0.2f;
    FResMarker.scale.z = 0.1f;
    FResMarker.color.a = 1.0f;
    FResMarker.color.r = 0.0f;
    FResMarker.color.g = 0.0f;
    FResMarker.color.b = 1.0f;
    //FResMarker.lifetime = dur;

    FMarkers.markers.push_back(FResMarker);
}

/**
 * Updates the FRep and FRes markers for visualization and puts them in the MarkerArray to publish them.
 */
void RobotController_Potential_Field::visualizeMarkers()
{
    // FRep
    // angle at which the nearest obstacle in this segment is located
    double angle = obstacles[1];
    // distance of the obstacle
    double dist = obstacles[0];
    // x-component of FRep for this segment
    double FRepX = FRep[0];
    // y-component of FRep for this segment
    double FRepY = FRep[1];

    // only do this if FRep != 0
    if(fabs(FRepX) > 0.0 || fabs(FRepY) > 0.0)
    {
        // arrow starts at the obstacle
        FrepMarker.pose.position.x = dist * cos(angle);
        FrepMarker.pose.position.y = dist * sin(angle);
        // orientation
        FrepMarker.pose.orientation.z = sin(atan2(FRepY, FRepX)/2.0);
        FrepMarker.pose.orientation.w = cos(atan2(FRepY, FRepX)/2.0);
        // length of arrow depends on FRep
        FrepMarker.scale.x = sqrt(FRepX*FRepX + FRepY*FRepY);
        FrepMarker.scale.y = 0.1f;
        FrepMarker.scale.z = 0.1f;
        FMarkers.markers.push_back(FrepMarker);
    } else // else reset arrows
    {
        FrepMarker.pose.position.x = 0.0f;
        FrepMarker.pose.position.y = 0.0f;
        FrepMarker.pose.orientation.z = 1.0f;
        FrepMarker.pose.orientation.w = 0.0f;
        FrepMarker.scale.x = 0.01f;
        FrepMarker.scale.y = 0.1f;
        FrepMarker.scale.z = 0.1f;
        FMarkers.markers.push_back(FrepMarker);
    }

    // FRes
    FResMarker.pose.orientation.z = sin(atan2(FResY, FResX)/2.0);
    FResMarker.pose.orientation.w = cos(atan2(FResY, FResX)/2.0);
    FResMarker.scale.x = sqrt(FResX*FResX + FResY*FResY);
    FMarkers.markers.push_back(FResMarker);

    F_pub.publish(FMarkers);

}

/**
 * updates the current potential field, i.e. all acting forces including the resulting force vector
 * input: the attractive force vector and the current laserscan from the robot
 */
void RobotController_Potential_Field::update(double newFAttX, double newFAttY)
{
    FAttX = newFAttX;
    FAttY = newFAttY;
    //compute the repulsive forces
    computeFReps();
    //compute the resulting force vector from both the repulsive and the attractive forces
    computeFRes();
}


/**
 * computes one repulsive force (coming from the closest obstacle) for each segment in the scanned area
 */
void RobotController_Potential_Field::computeFReps()
{
    //determine the closest obstacles in each segment
    findObstacles();
    //check all obstacles

    double min_dist = obstacles[0];
    //relative orientation of the obstacle to the robot
    double alpha = obstacles[1];
    //Is robot within obstacle's influence reach?
    if(min_dist <= opt_.dist_thresh() && min_dist > 0.0)
    {
        //polar position of the obstacle
        double obstX = min_dist * cos(alpha);
        double obstY = min_dist * sin(alpha);
        // computation of the repulsive force (x and y components)
        FRep[0] = opt_.kRep() * (1/min_dist - 1/opt_.dist_thresh()) * (1/(min_dist*min_dist)) * (-obstX/min_dist);
        FRep[1] = opt_.kRep() * (1/min_dist - 1/opt_.dist_thresh()) * (1/(min_dist*min_dist)) * (-obstY/min_dist);
    } else
    {
        FRep[0] = 0.0f;
        FRep[1] = 0.0f;
    }

}

/**
 * determines the nearest obstacle and stores it for further computation
 */
void RobotController_Potential_Field::findObstacles()
{
    double obst_angle = 0.0;
    auto obstacle_cloud = obstacle_avoider_->getObstacles();
    const pcl::PointCloud<pcl::PointXYZ>& cloud = *obstacle_cloud->cloud;
    pcl::PointCloud<pcl::PointXYZ> obst_points_new;
    double min_dist = std::numeric_limits<double>::infinity();
    if(cloud.header.frame_id == "base_link" || cloud.header.frame_id == "/base_link") {

        for(const pcl::PointXYZ& pt : cloud) {

            if(std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) < min_dist){

                obst_angle = std::atan2(pt.y, pt.x);
            }
            min_dist = std::min<double>(min_dist, std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z));
        }

    } else {
        tf::Transform trafo = pose_tracker_->getRelativeTransform(pose_tracker_->getRobotFrameId(), cloud.header.frame_id, ros::Time(0), ros::Duration(0));
        for(const pcl::PointXYZ& pt : cloud) {
            tf::Point pt_cloud(pt.x, pt.y, pt.z);
            tf::Point pt_robot = trafo * pt_cloud;

            if(pt_robot.length() < min_dist){
                obst_angle = std::atan2(pt_robot.getY(), pt_robot.getX());
            }

            min_dist = std::min<double>(min_dist, pt_robot.length());
        }
    }

    obstacles[0] = min_dist;
    obstacles[1] = obst_angle;
}

/**
 * computes the resulting force vector by combination of the attractive and repulsive forces
 */
void RobotController_Potential_Field::computeFRes()
{
    FResX = FAttX + FRep[0];
    FResY = FAttY + FRep[1];
}

void RobotController_Potential_Field::setGoalPosition()
{

    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();
    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];

    ///PATH FOLLOWING
    double theta = 0.0;
    double s_diff = std::numeric_limits<double>::max();
    // desired look-out distance wrt orthogonal projection
    double s_dist = 1.0;
    bool finished = false;
    tf::Point goal;

    if(fabs(fabs(fabs(path_interpol.s(proj_ind_) - path_interpol.s(path_interpol.n()-1)) - s_dist)) < 1e-1){
        theta = path_interpol.theta_p(path_interpol.n()-1) - theta_meas;
        tf::Point goal_tmp(path_interpol.p(path_interpol.n()-1), path_interpol.q(path_interpol.n()-1),0.0);
        goal = goal_tmp;
        finished = true;
    }

    for(int i = proj_ind_ + 1; i < path_interpol.n()-1 ; i++){
        if(fabs(fabs(path_interpol.s(proj_ind_) - path_interpol.s(i)) - s_dist) < s_diff && !finished){
            s_diff = fabs(fabs(path_interpol.s(proj_ind_) - path_interpol.s(i)) - s_dist);
            tf::Point goal_tmp(path_interpol.p(i), path_interpol.q(i), 0.0);
            goal = goal_tmp;
        }
    }

    mGoalPosX = goal.getX();
    mGoalPosY = goal.getY();

    double dx = mGoalPosX - x_meas;
    double dy = mGoalPosY - y_meas;

    FAttX = opt_.kAtt()*(dx*cos(-theta_meas) - dy*sin(-theta_meas));
    FAttY = opt_.kAtt()*(dx*sin(-theta_meas) + dy*cos(-theta_meas));

}


RobotController::MoveCommandStatus RobotController_Potential_Field::computeMoveCommand(MoveCommand *cmd)
{
    *cmd = MoveCommand(true);

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %d)", (int) path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }


    /// get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();

    const geometry_msgs::Twist v_meas_twist = pose_tracker_->getVelocity();

    double v_meas = getDirSign() * sqrt(v_meas_twist.linear.x * v_meas_twist.linear.x
                                        + v_meas_twist.linear.y * v_meas_twist.linear.y);

    double x_meas_ = current_pose[0];
    double y_meas_ = current_pose[1];
    double theta_meas_ = current_pose[2];
    ///***///


    // check for the subpaths, and see if the goal is reached
    if((proj_ind_ == path_interpol.n()-1) & (xe_ > 0.0)) {
        path_->switchToNextSubPath();
        // check if we reached the actual goal or just the end of a subpath
        if (path_->isDone()) {

            cmd_.speed = 0;
            cmd_.direction_angle = 0;
            cmd_.rotation = 0;

            *cmd = cmd_;

            double distance_to_goal_eucl = hypot(x_meas_ - path_interpol.p(path_interpol.n()-1),
                                                 y_meas_ - path_interpol.q(path_interpol.n()-1));

            ROS_INFO_THROTTLE(1, "Final positioning error: %f m", distance_to_goal_eucl);

            return RobotController::MoveCommandStatus::REACHED_GOAL;

        } else {
            //reset the Frenet-Serret frame
            proj_ind_ = 0;

            ROS_INFO("Next subpath...");
            // interpolate the next subpath
            path_interpol.interpolatePath(path_);
            publishInterpolatedPath();

            // recalculate the driving direction
            //calculateMovingDirection();
        }
    }

    //find the orthogonal projection to the curve and extract the corresponding index

    double dist = 0;
    double orth_proj = std::numeric_limits<double>::max();

    //quick solution for closed paths
    int old_ind = proj_ind_;

    for (unsigned int i = proj_ind_; i < path_interpol.n(); i++){

        dist = hypot(x_meas_ - path_interpol.p(i), y_meas_ - path_interpol.q(i));
        if(dist < orth_proj && std::abs(i - old_ind) < 50){

            orth_proj = dist;
            proj_ind_ = i;

        }

    }
    //***//

    ///calculate the control for the current point on the path

    //robot direction angle in path coordinates
    theta_e_ = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), theta_meas_);

    //robot position vector module
    double r = hypot(x_meas_ - path_interpol.p(proj_ind_), y_meas_ - path_interpol.q(proj_ind_));

    //robot position vector angle in world coordinates
    double theta_r = atan2(y_meas_ - path_interpol.q(proj_ind_), x_meas_ - path_interpol.p(proj_ind_));

    //robot position vector angle in path coordinates
    double delta_theta = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), theta_r);

    //current robot position in path coordinates
    xe_ = r * cos(delta_theta);
    ye_ = r * sin(delta_theta);

    ///***///

    // update markers put them in the MarkerArray to publish them
    FMarkers.markers.clear();

    // set the target point and compute FAtt
    setGoalPosition();

    // compute all the forces
    update(FAttX, FAttY);

    // FAtt marker
    marker.pose.orientation.z = sin(atan2(FAttY, FAttX)/2.0f);
    marker.pose.orientation.w = cos(atan2(FAttY, FAttX)/2.0f);

    double omega = 0.0;
    if(FResY * FResX != 0.0){
        omega = atan2(FResY, FResX);
    }
    cmd_.rotation = boost::algorithm::clamp(omega, -opt_.max_angular_velocity(), opt_.max_angular_velocity());


    marker.scale.x = sqrt(FAttX*FAttX + FAttY*FAttY);
    FMarkers.markers.push_back(marker);

    // FRep and FRes markers
    visualizeMarkers();

    cmd_.speed = getDirSign() * vn_;

    *cmd = cmd_;

    return MoveCommandStatus::OKAY;
}

void RobotController_Potential_Field::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}
