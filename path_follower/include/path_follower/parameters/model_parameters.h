#ifndef MODELPARAMETERS_H
#define MODELPARAMETERS_H

#include <path_follower/utils/parameters.h>
#include <rosconsole/macros_generated.h>
#include <path_follower/parameters/path_follower_parameters.h>
#include <model_based_planner/config_modelbasedplanner.h>

struct ModelParameters : public Parameters
{

    static const ModelParameters* getInstance()
    {
        static ModelParameters instance;
        return &instance;
    }




private:
    ros::NodeHandle nh;

public:

    P<std::string> robot_config_file;
    P<std::string> elevation_map_config_file;
    P<std::string> model_planner_type;


    //Planner
    P<int> max_num_nodes; //        int maxSearchIterations;
    P<int> max_depth; //        int maxLevel;
    P<int> curve_segment_subdivisions;//        int numSubSamples;
    P<double> look_ahead_time;//        float lookAheadTime;

    //

    // Scorer
    P<double> grav_angle_threshold; //float gravAngleThreshold;
    P<double> delta_angle_threshold; //float deltaAngleThreshold;
    P<double> tip_angle_threshold; //float tipAngleThreshold;
    P<double> min_wheel_support_threshold; //float minWheelSupportThreshold;
    P<bool> allow_not_visible; //bool allowNotVisible;
    P<double> no_wheel_support_near_threshold; //float noWheelSupportNearThreshold;
    P<double> no_wheel_support_rotate_threshold; //float noWheelSupportRotateThreshold;
    P<double> min_pose_steps_penalty_threshold; //float minPoseTime;

    P<double> score_weight_mean_grav_angle; //float f_meanGA;
    P<double> score_weight_max_grav_angle; //float f_maxGA;
    P<double> score_weight_mean_angle_difference; // float f_meanAD;
    P<double> score_weight_max_angle_difference; // float f_maxAD;
    P<double> score_weight_mean_tip_angle; // float f_meanTA;
    P<double> score_weight_max_tip_angle; // float f_maxTA;
    P<double> score_weight_mean_wheel_support; // float f_meanWS;
    P<double> score_weight_min_wheel_support; // float f_minWS;
    P<double> score_weight_not_visible_pose_count; //float f_numNotVisible;

    P<double> score_weight_delta_curvature; //float f_aVelD;
    P<double> score_weight_level; //float f_poseC;

    P<double> score_weight_distance_to_goal; // float f_goalDistance;
    P<double> score_weight_angle_to_goal; // float f_goalOrientation;
    P<double> score_weight_last_vel_diff; // float f_lastCmdVelDiff;


    P<double> score_weight_end_out_of_image; // float end_outOfImage;
    P<double> score_weight_end_no_wheel_support; //float end_noWheelSupport;
    P<double> score_weight_end_no_wheel_support_far; //float end_noWheelSupportFar;
    P<double> score_weight_end_not_visible; //float end_notVisible;
    P<double> score_weight_end_valid; //float end_valid;
    P<double> score_weight_end_exceed_angle; //float end_exceedAngle;
    P<double> score_weight_end_goal_reached; //float end_goalReached;

    P<double> score_weight_end_low_pose_count_penalty; //float end_poseCountLowPenalty;
    P<double> score_weight_end_chassis_collision; //float end_chassisCollision;

    P<double> target_goal_distance; //float targetGoalDistance;



    // Expander

    P<int> number_of_splits; //int numSplits;
    P<double> steering_angle_delta; //float deltaTheta;

    P<int> number_of_splits_first_level; //int firstLevelSplits;
    P<double> steering_angle_delta_first_level; //float firstLevelDeltaTheta;

    P<int> number_of_splits_first_level_linear; //int firstLevelSplits;
    P<double> linear_vel_delta_first_level; //float firstLevelDeltaTheta;


    void AssignParams(ModelBasedPlannerConfig &config)
    {
        config.plannerType_ = model_planner_type();


        //Planner
        config.plannerConfig_.maxSearchIterations = max_num_nodes();
        config.plannerConfig_.maxLevel = max_depth();
        config.plannerConfig_.numSubSamples = curve_segment_subdivisions();
        config.plannerConfig_.lookAheadTime = look_ahead_time();


        //Scorer
        config.scorerConfig_.gravAngleThreshold = grav_angle_threshold();
        config.scorerConfig_.deltaAngleThreshold = delta_angle_threshold();
        config.scorerConfig_.tipAngleThreshold = tip_angle_threshold();
        config.scorerConfig_.minWheelSupportThreshold = min_wheel_support_threshold();

        config.scorerConfig_.allowNotVisible = allow_not_visible();

        config.scorerConfig_.noWheelSupportNearThreshold = no_wheel_support_near_threshold();
        config.scorerConfig_.noWheelSupportRotateThreshold = no_wheel_support_rotate_threshold();
        config.scorerConfig_.minPoseTime = min_pose_steps_penalty_threshold();



        config.scorerConfig_.f_meanGA = score_weight_mean_grav_angle();
        config.scorerConfig_.f_maxGA = score_weight_max_grav_angle();
        config.scorerConfig_.f_meanAD = score_weight_mean_angle_difference();
        config.scorerConfig_.f_maxAD = score_weight_max_angle_difference();
        config.scorerConfig_.f_meanTA = score_weight_mean_tip_angle();
        config.scorerConfig_.f_maxTA = score_weight_max_tip_angle();
        config.scorerConfig_.f_meanWS = score_weight_mean_wheel_support();
        config.scorerConfig_.f_minWS = score_weight_min_wheel_support();
        config.scorerConfig_.f_numNotVisible = score_weight_not_visible_pose_count();

        config.scorerConfig_.f_aVelD = score_weight_delta_curvature();
        config.scorerConfig_.f_poseC = score_weight_level();

        config.scorerConfig_.f_goalDistance = score_weight_distance_to_goal();
        config.scorerConfig_.f_goalOrientation = score_weight_angle_to_goal();
        config.scorerConfig_.f_lastCmdVelDiff = score_weight_last_vel_diff();

        config.scorerConfig_.end_outOfImage = score_weight_end_out_of_image();
        config.scorerConfig_.end_noWheelSupport = score_weight_end_no_wheel_support();
        config.scorerConfig_.end_noWheelSupportFar = score_weight_end_no_wheel_support_far();
        config.scorerConfig_.end_notVisible = score_weight_end_not_visible();
        config.scorerConfig_.end_valid = score_weight_end_valid();
        config.scorerConfig_.end_exceedAngle = score_weight_end_exceed_angle();
        config.scorerConfig_.end_goalReached = score_weight_end_goal_reached();
        config.scorerConfig_.end_poseCountLowPenalty = score_weight_end_low_pose_count_penalty();
        config.scorerConfig_.end_chassisCollision = score_weight_end_chassis_collision();

        config.scorerConfig_.targetGoalDistance = target_goal_distance();


        //Expander
        config.expanderConfig_.numSplits = number_of_splits();
        config.expanderConfig_.deltaTheta = steering_angle_delta();

        config.expanderConfig_.firstLevelSplits = number_of_splits_first_level();
        config.expanderConfig_.firstLevelDeltaTheta = steering_angle_delta_first_level();

        config.expanderConfig_.firstLevelLinearSplits = number_of_splits_first_level_linear();
        config.expanderConfig_.firstLevelDeltaLinear = linear_vel_delta_first_level();


    }


    ModelParameters():
        Parameters("model_based_planner"),

        //Other local planner parameters
        robot_config_file(this, "robot_config_file", "", "Path to robot configuration file"),
        elevation_map_config_file(this, "elevation_map_config_file", "", "Path to elevation_map configuration file"),
        model_planner_type(this, "model_planner_type", "AStar_AngularVel_WSPL", "Type of model based planner used"),
        // Planner
        max_num_nodes(this, "max_num_nodes", 10000, "Determines the maximum number of nodes used by the local planner"),
        max_depth(this, "max_depth", 3, "Determines the maximum depth of the tree used by the local planner"),
        curve_segment_subdivisions(this, "curve_segment_subdivisions", 20, "Determines the number of subdivisions of curve segments in the final path"),
        look_ahead_time(this, "look_ahead_time", 3.0, "look ahead time for model based planner"),
        // Model based scores
        grav_angle_threshold(this, "grav_angle_threshold", 0.2, "Min value for angle between robot and gravity "),
        delta_angle_threshold(this, "delta_angle_threshold", 0.1, "Min value for angle between old and new robot pose "),
        tip_angle_threshold(this, "tip_angle_threshold", 0.1, "Min value for angle between the two robot poses "),
        min_wheel_support_threshold(this, "min_wheel_support_threshold", 0.5, "Min value for wheel support "),
        allow_not_visible(this, "allow_not_visible", false, "Allow drive over not yet observed areas? "),
        no_wheel_support_near_threshold(this, "no_wheel_support_near_threshold", 50.0, "Min value for wheel support "),
        no_wheel_support_rotate_threshold(this, "no_wheel_support_rotate_threshold", 0.7, "Min value for wheel support "),
        min_pose_steps_penalty_threshold(this, "min_pose_steps_penalty_threshold", 20.0, "Min value for wheel support "),
        score_weight_mean_grav_angle(this, "score_weight_mean_grav_angle", -1.0, "Determines whether the mean gravityangle score is used or not. (Angle between robot and gravity)"),
        score_weight_max_grav_angle(this, "score_weight_max_grav_angle", -0.5, "Determines whether the max gravityangle score is used or not. (Angle between robot and gravity)"),
        score_weight_mean_angle_difference(this, "score_weight_mean_angle_difference", -10.0, "Determines whether the mean angle difference score is used or not. (Angle between robot and gravity)"),
        score_weight_max_angle_difference(this, "score_weight_max_angle_difference", -1.0, "Determines whether the max angle difference score is used or not. (Angle between robot and gravity)"),
        score_weight_mean_tip_angle(this, "score_weight_mean_tip_angle", -1.0, "Determines whether the mean tipangle score is used or not. (Angle between robot and gravity)"),
        score_weight_max_tip_angle(this, "score_weight_max_tip_angle", -0.5, "Determines whether the max tipangle score is used or not. (Angle between robot and gravity)"),
        score_weight_mean_wheel_support(this, "score_weight_mean_wheel_support", 2.0, "Determines whether the mean wheel support score is used or not. (Angle between robot and gravity)"),
        score_weight_min_wheel_support(this, "score_weight_min_wheel_support", 2.0, "Determines whether the min wheel support score is used or not. (Angle between robot and gravity)"),
        score_weight_not_visible_pose_count(this, "score_weight_not_visible_pose_count", 2.0, " min number poses for not applying penalty"),
        score_weight_delta_curvature(this,"score_weight_delta_curvature",0.0,"Determines whether the fouth scorer is used or not. (Curvature of the point (D))"),
        score_weight_level(this,"score_weight_level", 0.0, "Determines whether the fifth scorer is used or not. (Tree level reached)"),
        score_weight_distance_to_goal(this,"score_weight_distance_to_goal",-1.0,"Determines whether the first scorer is used or not. (Distance to global path (P))"),
        score_weight_angle_to_goal(this,"score_weight_angle_to_goal",-1.0,"Determines whether the third scorer is used or not. (Curvature of the point (P))"),
        score_weight_last_vel_diff(this,"score_weight_last_vel_diff",-1.0,"Difference to last command velocity"),
        score_weight_end_out_of_image(this,"score_weight_end_out_of_image",-1.0,"score_weight_end_out_of_image"),
        score_weight_end_no_wheel_support(this,"score_weight_end_no_wheel_support",-100.0,"score_weight_end_no_wheel_support"),
        score_weight_end_no_wheel_support_far(this,"score_weight_end_no_wheel_support_far",-10.0,"score_weight_end_no_wheel_support"),
        score_weight_end_not_visible(this,"score_weight_end_not_visible",-10.0,"score_weight_end_not_visible"),
        score_weight_end_valid(this,"score_weight_end_valid",0.0,"score_weight_end_valid"),
        score_weight_end_exceed_angle(this,"score_weight_end_exceed_angle",-1000.0,"score_weight_end_exceed_angle"),
        score_weight_end_goal_reached(this,"score_weight_end_goal_reached",100.0,"score_weight_end_goal_reached"),
        score_weight_end_low_pose_count_penalty(this,"score_weight_end_low_pose_count_penalty",-100.0,"score_weight_end_goal_reached"),
        score_weight_end_chassis_collision(this,"score_weight_end_chassis_collision",-1000.0,"score_weight_end_chassis_collision"),
        target_goal_distance(this,"target_goal_distance",0.2,"score_weight_end_goal_reached"),
        // expander
        number_of_splits(this, "number_of_splits", 3, "number_of_splits for node expansion"),
        steering_angle_delta(this, "steering_angle_delta", 0.4, "Angle change for node expansion"),
        number_of_splits_first_level(this, "number_of_splits_first_level", 7, "number_of_splits for node expansion"),
        steering_angle_delta_first_level(this, "steering_angle_delta_first_level", 0.2, "Angle change for node expansion"),
        number_of_splits_first_level_linear(this, "number_of_splits_first_level_linear", 3, "number_of_linear velocity splits for node expansion"),
        linear_vel_delta_first_level(this, "linear_vel_delta_first_level", 0.1, "Angle change for node expansion")



      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {

    }
};

#endif // MODELPARAMETERS_H
