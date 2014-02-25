/*
 * sbpl_planner_node.hpp
 *
 *  Created on: Feb 24, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef SBPL_PLANNER_NODE_H
#define SBPL_PLANNER_NODE_H

/// COMPONENT
#include "planner_node.h"

/// PROJECT
#include <utils_path/common/SimpleGridMap2d.h>
#include <utils_path/common/Path.h>

/// SYSTEM
#include <nav_msgs/Path.h>
#include <sbpl/headers.h>
#include <sbpl/planners/planner.h>
#include <ros/package.h>

using namespace lib_path;

/**
 * @brief The SBPLPathPlanner struct uses the Search-Based-Planning-Library to plan paths
 */
struct SBPLPathPlanner : public Planner
{
    SBPLPathPlanner()
    {
        std::string path = ros::package::getPath("path_planner");

        motPrimFilename = path + "/cfg/motion.mprim";
        nh.param("mot_prim_filename", motPrimFilename, motPrimFilename);

        nh.param("half_width", half_width_, 0.15);
        nh.param("half_length", half_length_, 0.2);

        createFootprint();
    }

    void initEnvironment(EnvironmentNAVXYTHETALAT& env) {

        int width = map_info->getWidth();
        int height = map_info->getHeight();
        double cellsize_m = map_info->getResolution();

        // initialize an environment
        double nominalvel_mpersecs = 1.0;
        double timetoturn45degsinplace_secs = 0;
        unsigned char obsthresh = 10;

        params.mapdata = map_info->getData();

        params.goaltol_x = 0.1;
        params.goaltol_y = 0.1;
        params.goaltol_theta = 0.05;

        params.numThetas = 16;

        env.InitializeEnv(width, height, perimeter, cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh, motPrimFilename.c_str(), params);
    }

    // creating the footprint
    void createFootprint(){
        sbpl_2Dpt_t pt_m;
        pt_m.x = -half_length_;
        pt_m.y = -half_width_;
        perimeter.push_back(pt_m);
        pt_m.x = half_length_;
        pt_m.y = -half_width_;
        perimeter.push_back(pt_m);
        pt_m.x = half_length_;
        pt_m.y = half_width_;
        perimeter.push_back(pt_m);
        pt_m.x = -half_length_;
        pt_m.y = half_width_;
        perimeter.push_back(pt_m);

        std::cout << "footprint is (w x h) " << half_width_*2 << " x " << half_length_ *2 << std::endl;
    }

    boost::shared_ptr<SBPLPlanner> initializePlanner(EnvironmentNAVXYTHETALAT& env,
                                                     int start_id, int goal_id,
                                                     double initialEpsilon,
                                                     bool bsearchuntilfirstsolution){
        bool bsearch = false;
        boost::shared_ptr<SBPLPlanner> planner(new ARAPlanner(&env, bsearch));
        // set planner properties
        if (planner->set_start(start_id) == 0) {
            printf("ERROR: failed to set start state\n");
            throw new SBPL_Exception();
        }
        if (planner->set_goal(goal_id) == 0) {
            printf("ERROR: failed to set goal state\n");
            throw new SBPL_Exception();
        }
        planner->set_initialsolution_eps(initialEpsilon);
        planner->set_search_mode(bsearchuntilfirstsolution);

        return planner;
    }

    void convertSolution(EnvironmentNAVXYTHETALAT& env, vector<int> solution_stateIDs,
                         nav_msgs::Path& path){
        vector<sbpl_xy_theta_pt_t> xythetaPath;
        env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
        for (unsigned int i = 0; i < xythetaPath.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = xythetaPath.at(i).x + map_info->getOrigin().x;
            pose.pose.position.y = xythetaPath.at(i).y + map_info->getOrigin().y;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(xythetaPath.at(i).theta);//DiscTheta2Cont(theta, 16));
            path.poses.push_back(pose);
        }
    }

    void plan(const geometry_msgs::PoseStamped &goal,
              const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
              const lib_path::Pose2d&, const lib_path::Pose2d&)
    {
        params.startx = from_world.x - map_info->getOrigin().x;
        params.starty = from_world.y - map_info->getOrigin().y;
        params.starttheta = from_world.theta;

        params.goalx = to_world.x - map_info->getOrigin().x;
        params.goaly = to_world.y - map_info->getOrigin().y;
        params.goaltheta = to_world.theta;

        EnvironmentNAVXYTHETALAT env;
        initEnvironment(env);

        int start_id = env.SetStart(params.startx, params.starty, params.starttheta);
        int goal_id = env.SetGoal(params.goalx, params.goaly, params.goaltheta);

        // initialize a planner with start and goal state
        double initialEpsilon = 3.0;
        bool bsearchuntilfirstsolution = false;
        boost::shared_ptr<SBPLPlanner> planner = initializePlanner(env, start_id, goal_id, initialEpsilon,
                                                                   bsearchuntilfirstsolution);

        // plan
        vector<int> solution_stateIDs;
        double allocated_time_secs = 10.0; // in seconds
        planner->replan(allocated_time_secs, &solution_stateIDs);

        // print stats
        env.PrintTimeStat(stdout);

        // publish solution
        nav_msgs::Path path;
        path.header.frame_id = goal.header.frame_id;
        path.header.stamp = goal.header.stamp;
        convertSolution(env, solution_stateIDs, path);

        publish(path);
    }

private:
    std::vector<sbpl_2Dpt_t> perimeter;
    std::string motPrimFilename;

    double half_width_;
    double half_length_;

    EnvNAVXYTHETALAT_InitParms params;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");

    SBPLPathPlanner planner;

    ros::WallRate r(30);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}

#endif // SBPL_PLANNER_NODE_H
