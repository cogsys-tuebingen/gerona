#ifndef ROBOTCONTROLLER_KINEMATIC_HBZ_TT_CPP
#define ROBOTCONTROLLER_KINEMATIC_HBZ_TT_CPP

// HEADER
#include <path_follower/controller/robotcontroller_kinematic_HBZ_TT.h>

// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/collision_avoidance/collision_avoider.h>
#include <path_follower/factory/controller_factory.h>


// SYSTEM
//#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(RobotController_Kinematic_HBZ_TT, RobotController)

REGISTER_ROBOT_CONTROLLER(RobotController_Kinematic_HBZ_TT, kinematic_hbz_TT, default_collision_avoider);


using namespace Eigen;


RobotController_Kinematic_HBZ_TT::RobotController_Kinematic_HBZ_TT():
    RobotController_Kinematic_HBZ(),
    counter_(1),
    mean_vel_(0)
{
    meas_velocities_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    previous_t_ = ros::Time::now();
    last_movement_ = ros::Time::now();
    target_vec_prev_ = tf::Vector3(0,0,0);
}

double RobotController_Kinematic_HBZ_TT::computeSpeed()
{
    ///get the person's velocity
    tf::Transform target_tf_world = pose_tracker_->getTransform(getFixedFrame(), "target", ros::Time::now(), ros::Duration(0.01));
    tf::Vector3 target_vec = target_tf_world.getOrigin();


    if(std::abs(target_vec.length() - target_vec_prev_.length()) > 1e-1){
        ros::Time current_t = ros::Time::now();
        double dt = (current_t - previous_t_).toSec();
        previous_t_ = current_t;
        tf::Vector3 target_v_vec = (target_vec - target_vec_prev_)/std::max(1e-5,dt);
        target_vec_prev_ = target_vec;
        double target_v = target_v_vec.length()  <= vn_ ? target_v_vec.length() : vn_;

        //compute the mean value using the "meas_velocities.size()" last measured velocities
        //e.g. if meas_velocities_ = {0, 0}, the mean over the last two values is used, i.e. mean_vel_ = (i[t] + i[t-1])/2.0
        meas_velocities_[counter_] = target_v;
        mean_vel_ = 0.0;
        int meas_num = meas_velocities_.size();
        for(double i : meas_velocities_){
            mean_vel_ += i;
            if(i < 1e-5){
                meas_num--;
            }
        }
        mean_vel_ /= meas_num;

        if(counter_ == meas_velocities_.size() - 1){
            counter_ = -1;
        }
        counter_++;

        last_movement_ = ros::Time::now();
    }
    ///***///

    ///get the distance to the target in path coordinates
    //find the orthogonal projection to the curve and extract the corresponding index

    double dist_glob = 0;
    double orth_proj_glob = std::numeric_limits<double>::max();
    uint proj_ind_glob = 0;

    for (unsigned int i = proj_ind_glob; i < global_path_.n(); i++){

        dist_glob = hypot(x_meas_ - global_path_.p(i), y_meas_ - global_path_.q(i));
        if(dist_glob < orth_proj_glob){
            orth_proj_glob = dist_glob;
            proj_ind_glob = i;
        }
    }

    double dist_to_goal_glob = global_path_.s(global_path_.n()-1) - global_path_.s(proj_ind_glob);

    ///***///

    ///calculate the necessary speed to reach and keep the desired position

    //if the person doesn't move for 3 seconds, come to 1m distance
    double time_threshold = 3.0;
    double v_d = 0;
    if((fabs((ros::Time::now() - last_movement_).toSec()) > time_threshold) && mean_vel_ < 1e-3){
        v_d = 1/std::cos(theta_e_)*(mean_vel_ - opt_.x_ICR()*angular_vel_*sin(theta_e_) - opt_.k1() * xe_
                                 + opt_.k_l()*(dist_to_goal_glob - 1.0));
    }
    else{
        v_d = 1/std::cos(theta_e_)*(mean_vel_ - opt_.x_ICR()*angular_vel_*sin(theta_e_) - opt_.k1() * xe_
                                           + opt_.k_l()*(dist_to_goal_glob - opt_.des_dist()));
    }
    ///***///

    double v = std::min(v_d, vn_);
    v = v > 0.0 ? v : 0.0;


    //HBZ Lyapunov speed control, where the input speed is the distance-controlled speed, instead of vn_

    double vdc = v;

    double V1 = 1.0/2.0*(std::pow(xe_,2) + std::pow(ye_,2) + 1.0/opt_.lambda()*fabs(sin(theta_e_-delta_)));

    if(angular_vel_ > 0){

        if(V1 >= opt_.epsilon()){
            v = (-opt_.alpha_r()*opt_.y_ICR_l()*vdc)/(opt_.y_ICR_r() - opt_.y_ICR_l());
        }
        else if(V1 < opt_.epsilon()){
            v = (opt_.alpha_r()*vdc)/(1 + std::fabs(opt_.y_ICR_r()*path_interpol.curvature(ind_)));
        }
    }
    else if(angular_vel_ <= 0){

        if(V1 >= opt_.epsilon()){
            v = (opt_.alpha_l()*opt_.y_ICR_r()*vdc)/(opt_.y_ICR_r() - opt_.y_ICR_l());
        }
        else if(V1 < opt_.epsilon()){
            v = (opt_.alpha_l()*vdc)/(1 + std::fabs(opt_.y_ICR_l()*path_interpol.curvature(ind_)));
        }
    }

    return v;

}


#endif // ROBOTCONTROLLER_KINEMATIC_HBZ_TT_CPP

