// HEADER
#include <path_follower/controller/robotcontroller_dynamic_window.h>

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
#include <boost/tuple/tuple.hpp>

using namespace Eigen;
using namespace std;

RobotController_Dynamic_Window::RobotController_Dynamic_Window():
    cmd_(this),
    vn_(0.0),
    proj_ind_(0),
    xe_(0.0),
    ye_(0.0),
    theta_e_(0.0),
    v_cmd_(0.0),
    w_cmd_(0.0),
    curv_dist_obst_(1e5),
    mGoalPosX(0.0),
    mGoalPosY(0.0),
    x_meas_(0.0),
    y_meas_(0.0),
    theta_meas_(0.0),
    theta_pred_(0.0),
    v_iter_(0.0),
    w_iter_(0.0),
    x_pred_(0.0),
    y_pred_(0.0),
    x_next_(0.0),
    y_next_(0.0),
    theta_next_(0.0),
    obstacle_found(false),
    m_id_counter(0)
{
    t_old_ = ros::Time::now();
    goal_pub = nh_.advertise<geometry_msgs::PointStamped>("goal_position", 0);
    far_pred_pub = nh_.advertise<visualization_msgs::MarkerArray>("far_predicted_positions", 0);
    predict_pub = nh_.advertise<geometry_msgs::PointStamped>("predicted_position", 0);
    obst_marker_pub = nh_.advertise<visualization_msgs::Marker>("obstacle_distance", 0);
    obst_point_pub = nh_.advertise<geometry_msgs::PointStamped>("obstacle_point", 0);
    traj_pub = nh_.advertise<nav_msgs::Path>("possible_trajectories", 0);
}


void RobotController_Dynamic_Window::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Dynamic_Window::initialize()
{
    RobotController::initialize();

    // desired velocity
    vn_ = std::min(global_opt_->max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);
}

void RobotController_Dynamic_Window::reset()
{
    RobotController::reset();
}


void RobotController_Dynamic_Window::setPath(Path::Ptr path)
{
    RobotController::setPath(path);

    //initial velocity commands
    v_cmd_ = 0.5;
    w_cmd_ = 1e-3;
}


void RobotController_Dynamic_Window::setGoalPosition()
{

    ///PATH FOLLOWING
    double theta = 0.0;
    double s_diff = std::numeric_limits<double>::max();
    // desired look-out distance wrt orthogonal projection
    double s_dist = 2.0;
    bool finished = false;
    tf::Point goal;

    if(fabs(fabs(fabs(path_interpol.s(proj_ind_) - path_interpol.s(path_interpol.n()-1)) - s_dist)) < 1e-1){
        theta = path_interpol.theta_p(path_interpol.n()-1) - theta_meas_;
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

    geometry_msgs::PointStamped goal_pose;
    goal_pose.point.x = mGoalPosX;
    goal_pose.point.y = mGoalPosY;
    goal_pose.header.frame_id = pose_tracker_->getFixedFrameId();
    goal_pub.publish(goal_pose);
}



void RobotController_Dynamic_Window::searchMinObstDist(){

    visualization_msgs::Marker obst_dist_marker;
    obst_dist_marker.header.frame_id = pose_tracker_->getFixedFrameId();
    obst_dist_marker.header.stamp = ros::Time();
    obst_dist_marker.ns = "obstacle_distance";
    obst_dist_marker.id = 1445;
    obst_dist_marker.type = visualization_msgs::Marker::ARROW;
    obst_dist_marker.action = visualization_msgs::Marker::ADD;

    auto obstacle_cloud = obstacle_avoider_->getObstacles();
    const pcl::PointCloud<pcl::PointXYZ>& cloud = *obstacle_cloud->cloud;
    double min_dist = std::numeric_limits<double>::infinity();
    tf::Point coll_pt(0.0, 0.0, 0.0);
    if(cloud.header.frame_id == pose_tracker_->getFixedFrameId()) {
        for(const pcl::PointXYZ& pt : cloud) {
            if(std::hypot(pt.x - x_pred_, pt.y - y_pred_) < min_dist){
                min_dist = std::hypot(pt.x - x_pred_, pt.y - y_pred_);
                coll_pt.setX(pt.x);
                coll_pt.setY(pt.y);
            }
        }

    } else {
        tf::Transform trafo = pose_tracker_->getRelativeTransform(pose_tracker_->getFixedFrameId(), cloud.header.frame_id, ros::Time(0), ros::Duration(0));
        for(const pcl::PointXYZ& pt : cloud) {
            tf::Point pt_cloud(pt.x, pt.y, pt.z);
            tf::Point pt_ff = trafo * pt_cloud;
            if(std::hypot(pt_ff.getX() - x_pred_, pt_ff.getY() - y_pred_)){
                min_dist = std::hypot(pt_ff.getX() - x_pred_, pt_ff.getY() - y_pred_);
                coll_pt.setX(pt_ff.getX());
                coll_pt.setY(pt_ff.getY());
            }
        }
    }


    if(min_dist > opt_.obst_dist_thresh()){
        curv_dist_obst_ = 10.0;
        obst_dist_marker.points.clear();
    }
    else{

        geometry_msgs::Point p1, p2;

        p1.x = x_pred_;
        p1.y = y_pred_;
        p2.x = coll_pt.getX();
        p2.y = coll_pt.getY();

        obst_dist_marker.pose.position.x = coll_pt.getX();
        obst_dist_marker.pose.position.y = coll_pt.getY();
        obst_dist_marker.pose.position.z = 0.0;

        tf::Quaternion quaternion = tf::createQuaternionFromYaw(std::atan2(y_pred_-coll_pt.getY(), x_pred_-coll_pt.getX()));
        obst_dist_marker.pose.orientation.x = quaternion.getX();
        obst_dist_marker.pose.orientation.y = quaternion.getY();
        obst_dist_marker.pose.orientation.z = quaternion.getZ();
        obst_dist_marker.pose.orientation.w = quaternion.getW();
        obst_dist_marker.scale.x = std::hypot(y_pred_-coll_pt.getY(), x_pred_-coll_pt.getX());
        obst_dist_marker.scale.y = 0.1f;
        obst_dist_marker.scale.z = 0.1f;
        obst_dist_marker.color.a = 1.0f;
        obst_dist_marker.color.r = 0.0f;
        obst_dist_marker.color.g = 1.0f;
        obst_dist_marker.color.b = 0.0f;

        if(std::abs(w_iter_) < 1e-1){
            curv_dist_obst_ = std::hypot(y_next_ - y_pred_, x_next_ - x_pred_);
        }
        else{
            //current curvature radius
            double r = v_iter_/w_iter_;
            //current center of the circle
            double Cx = x_next_ - r * std::sin(theta_next_);
            double Cy = y_next_ + r * std::cos(theta_next_);

            //vector from the center of the circle to the predicted robot position
            Vector2d vec_rob(x_next_ - Cx, y_next_ - Cy);
            //vector from the center of the circle to the far predicted collision point
            Vector2d vec_coll(x_pred_ - Cx, y_pred_ - Cy);
            //angle difference between the two vectors
            double angle_diff = MathHelper::Angle(vec_rob, vec_coll);

            //compute the distance on the arc to the nearest obstacle
            curv_dist_obst_ = std::abs(r * angle_diff);
        }

        obstacle_found = true;
    }
    obst_marker_pub.publish(obst_dist_marker);

}


bool RobotController_Dynamic_Window::checkAdmissibleVelocities(){
    double x_new = 0.0;
    double y_new = 0.0;
    double theta_new = theta_meas_;
    double t = opt_.step_T();
    double t_count = 0.0;
    x_pred_ = x_meas_;
    y_pred_ = y_meas_;
    x_next_ = x_meas_;
    y_next_ = y_meas_;
    theta_next_ = theta_meas_;


    m_id_counter++;
    visualization_msgs::Marker far_pred_point;
    far_pred_point.points.clear();
    far_pred_point.header.frame_id = pose_tracker_->getFixedFrameId();
    far_pred_point.header.stamp = ros::Time::now();
    far_pred_point.ns = "far_predictions";
    far_pred_point.id = 1446 + m_id_counter;
    far_pred_point.type = visualization_msgs::Marker::LINE_STRIP;
    far_pred_point.action = visualization_msgs::Marker::ADD;
    far_pred_point.pose.orientation.w = 1.0;
    far_pred_point.scale.x = 0.05;
    far_pred_point.scale.y = 0.05;
    far_pred_point.scale.z = 0.1f;
    far_pred_point.color.a = 1.0f;
    far_pred_point.color.r = 0.0f;
    far_pred_point.color.g = 1.0f;
    far_pred_point.color.b = 0.0f;

    geometry_msgs::PointStamped obst_point;
    obst_point.header.frame_id = pose_tracker_->getFixedFrameId();

    if(std::abs(w_iter_) < 1e-1){
        while(t_count < opt_.fact_T()*opt_.T_dwa()){
            t_count += opt_.step_T();
            theta_new += w_iter_*opt_.step_T();
            x_new = v_iter_ * std::cos(theta_new) * t;
            y_new = v_iter_ * std::sin(theta_new) * t;
            x_pred_ += x_new;
            y_pred_ += y_new;
            obstacle_found = false;
            searchMinObstDist();
            x_next_ = x_pred_;
            y_next_ = y_pred_;
            theta_next_ = theta_new;

            geometry_msgs::Point p;
            p.x = x_pred_;
            p.y = y_pred_;
            far_pred_point.points.push_back(p);

            traj_.header.frame_id = pose_tracker_->getFixedFrameId();
            geometry_msgs::PoseStamped pos_st;
            pos_st.pose.position.x = x_pred_;
            pos_st.pose.position.y = y_pred_;
            traj_.poses.push_back(pos_st);
            traj_pub.publish(traj_);


            if(obstacle_found){
                obst_point.point.x = x_pred_;
                obst_point.point.y = y_pred_;
                break;
            }
            if(std::abs(t_count - opt_.T_dwa()) < 1e-1){

                double goal_angle = std::atan2(mGoalPosY - y_pred_, mGoalPosX - x_pred_);
                theta_pred_ = MathHelper::AngleDelta(goal_angle, theta_new);

                geometry_msgs::PointStamped next_pos;
                next_pos.point.x = x_pred_;
                next_pos.point.y = y_pred_;
                next_pos.header.frame_id = pose_tracker_->getFixedFrameId();
                predict_pub.publish(next_pos);
            }
        }
    }
    else{
        while(t_count < opt_.fact_T()*opt_.T_dwa()){
            x_new = v_iter_/w_iter_ * (std::sin(theta_new + w_iter_*opt_.step_T()) - std::sin(theta_new));
            y_new = v_iter_/w_iter_ * (std::cos(theta_new) - std::cos(theta_new + w_iter_*opt_.step_T()));
            x_pred_ += x_new;
            y_pred_ += y_new;
            t_count += opt_.step_T();
            theta_new += w_iter_*opt_.step_T();
            obstacle_found = false;
            searchMinObstDist();
            x_next_ = x_pred_;
            y_next_ = y_pred_;
            theta_next_ = theta_new;

            geometry_msgs::Point p;
            p.x = x_pred_;
            p.y = y_pred_;
            far_pred_point.points.push_back(p);

            traj_.header.frame_id = pose_tracker_->getFixedFrameId();
            geometry_msgs::PoseStamped pos_st;
            pos_st.pose.position.x = x_pred_;
            pos_st.pose.position.y = y_pred_;
            traj_.poses.push_back(pos_st);
            traj_pub.publish(traj_);

            if(obstacle_found){
                obst_point.point.x = x_pred_;
                obst_point.point.y = y_pred_;
                break;
            }
            if(std::abs(t_count - opt_.T_dwa()) < 1e-1){

                double goal_angle = std::atan2(mGoalPosY - y_pred_, mGoalPosX - x_pred_);
                theta_pred_ = MathHelper::AngleDelta(goal_angle, theta_new);

                geometry_msgs::PointStamped next_pos;
                next_pos.point.x = x_pred_;
                next_pos.point.y = y_pred_;
                next_pos.header.frame_id = pose_tracker_->getFixedFrameId();
                predict_pub.publish(next_pos);
            }
        }
    }

    obst_point_pub.publish(obst_point);

    if((v_iter_ <= std::sqrt(2.0 * curv_dist_obst_ * opt_.lin_dec())) && (std::abs(w_iter_) <= std::sqrt(2.0 * curv_dist_obst_ * opt_.ang_dec()))){
        far_pred_points.markers.push_back(far_pred_point);
        far_pred_pub.publish(far_pred_points);

        return true;
    }
    else{
        return false;
    }
}

void RobotController_Dynamic_Window::findNextVelocityPair()
{
    double v_wind_b = std::max(0.0, v_cmd_ - opt_.lin_acc()*opt_.T_dwa());
    double v_wind_t = std::min((double)global_opt_->max_velocity(), v_cmd_ + opt_.lin_acc()*opt_.T_dwa());

    double w_wind_l = std::max(-opt_.max_ang_vel(), w_cmd_ - opt_.ang_acc()*opt_.T_dwa());
    double w_wind_r = std::min(opt_.max_ang_vel(), w_cmd_ + opt_.ang_acc()*opt_.T_dwa());

    v_iter_ = v_wind_b - opt_.v_step();
    w_iter_ = w_wind_l - opt_.w_step();

    vector< tuple<double,double,double> > vels_and_objfunc;

    far_pred_points.markers.clear();
    visualization_msgs::Marker clearing_marker;
    clearing_marker.header.frame_id = pose_tracker_->getFixedFrameId();
    clearing_marker.header.stamp = ros::Time::now();
    clearing_marker.ns = "far_predictions";
    clearing_marker.id = 0;
#if ROS_VERSION_MINIMUM(1, 11, 21)
    clearing_marker.action = visualization_msgs::Marker::DELETEALL;
#else
    clearing_marker.action = 3u;
#endif
    far_pred_points.markers.push_back(clearing_marker);
    m_id_counter = 0;
    traj_.poses.clear();

    while(v_iter_ < v_wind_t){
        v_iter_ += opt_.v_step();
        while(w_iter_ < w_wind_r){
            w_iter_ += opt_.w_step();
            if(checkAdmissibleVelocities()){
                double heading = 1.0 - std::abs(theta_pred_)/M_PI;
                double obj_func = opt_.angle_fact()*heading + opt_.disobst_fact()*curv_dist_obst_ + opt_.v_fact()*v_iter_;
                vels_and_objfunc.push_back(tuple<double,double,double>(obj_func,v_iter_,w_iter_));
            }
        }
        w_iter_ = w_wind_l - opt_.w_step();
    }

    double max_obj = std::numeric_limits<double>::min();

    if(vels_and_objfunc.size() < 1){
        std::cout << "There are no admissible velocities!!!" << std::endl;
    }

    for(uint i = 0; i < vels_and_objfunc.size(); i++){
        if(get<0>(vels_and_objfunc[i]) > max_obj){
            max_obj = get<0>(vels_and_objfunc[i]);
            v_cmd_ = boost::algorithm::clamp(get<1>(vels_and_objfunc[i]), 0.0, global_opt_->max_velocity());
            w_cmd_ = boost::algorithm::clamp(get<2>(vels_and_objfunc[i]), -opt_.max_ang_vel(), opt_.max_ang_vel());
        }
    }

}


RobotController::MoveCommandStatus RobotController_Dynamic_Window::computeMoveCommand(MoveCommand *cmd)
{
    *cmd = MoveCommand(true);

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %d)", (int) path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }

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

    //set the current goal position
    setGoalPosition();

    //find the next velocity pair
    if((ros::Time::now() - t_old_).toSec() >= opt_.T_dwa()){

        /// get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
        Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();
        x_meas_ = current_pose[0];
        y_meas_ = current_pose[1];
        theta_meas_ = current_pose[2];

        x_pred_ = x_meas_;
        y_pred_ = y_meas_;

        findNextVelocityPair();
        t_old_ = ros::Time::now();
    }

    cmd_.rotation = boost::algorithm::clamp(w_cmd_, -opt_.max_ang_vel(), opt_.max_ang_vel());
    cmd_.speed = getDirSign() * v_cmd_;

    *cmd = cmd_;

    return MoveCommandStatus::OKAY;
}

void RobotController_Dynamic_Window::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

