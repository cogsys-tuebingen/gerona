/*
 * ompl_planner_node.hpp
 *
 *  Created on: Feb 20, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// COMPONENT
#include "planner_node.h"

/// SYSTEM
#include <iostream>
#include <nav_msgs/Path.h>
#include <omplapp/apps/KinematicCarPlanning.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/config.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace ompl;

/**
 * @brief The OMPLPlanner struct uses the Open-Motion-Planning-Library to plan paths
 */
struct OMPLPlanner : public Planner
{
    OMPLPlanner()
    {
        // plan for kinematic car in SE(2)
        base::StateSpacePtr SE2(setup.getStateSpace());

        // set the bounds for the R^2 part of SE(2)
        base::RealVectorBounds bounds(2);
        bounds.setLow(-10);
        bounds.setHigh(10);
        SE2->as<base::SE2StateSpace>()->setBounds(bounds);

        setup.setVehicleLength(0.5);

        setup.setStateValidityChecker(boost::bind(&OMPLPlanner::isStateValid, this, _1));
        setup.setPlanner(base::PlannerPtr(new control::EST(setup.getSpaceInformation())));
        //setup.setPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
        //setup.setPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));
        //setup.setPlanner(base::PlannerPtr(new control::PDST(setup.getSpaceInformation())));
    }

private:

    bool isStateValid(const ob::State *state) const
    {
        const base::SE2StateSpace::StateType* s = state->as<base::SE2StateSpace::StateType>();

        double px = s->getX();
        double py = s->getY();

        unsigned x, y;
        map_info->point2cell(px, py, x, y);

        return map_info->isInMap((int) x,(int) y) && map_info->isFree(x,y);
    }

    void plan (const geometry_msgs::PoseStamped &goal,
               const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
               const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {
        setup.clear();

        ob::ScopedState<> start_state(setup.getStateSpace());
        start_state[0] = from_world.x;
        start_state[1] = from_world.y;
        start_state[2] = from_world.theta;
        ob::ScopedState<> goal_state(setup.getStateSpace());
        goal_state[0] = to_world.x;
        goal_state[1] = to_world.y;
        goal_state[2] = to_world.theta;
        setup.setStartAndGoalStates(start_state, goal_state, 0.3);

        //setup.setStatePropagator(boost::bind(&propagate, setup.getSpaceInformation().get(), _1, _2, _3, _4));

        control::SpaceInformationPtr si = setup.getSpaceInformation();
        si->setPropagationStepSize(.05);
        si->setMinMaxControlDuration(1, 10);

        ob::ProblemDefinitionPtr pdef(setup.getProblemDefinition());
        pdef->setOptimizationObjective(getPathLengthObjective(si));



        std::vector<double> cs(2);
        cs[0] = cs[1] = 0.1;
        setup.setup();

        setup.getPlanner()->setProblemDefinition(pdef);
        setup.getPlanner()->clear();
        setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

        setup.solve(5);

        const std::size_t ns = setup.getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (setup.haveSolutionPath()) {
            //            control::PathControl& p(setup.getSolutionPath());
            geometric::PathGeometric p(setup.getSolutionPath().asGeometric());

            p.printAsMatrix(std::cout);
            if (!setup.haveExactSolutionPath())
            {
                std::cout << "Solution is approximate. Distance to actual goal is " <<
                             setup.getProblemDefinition()->getSolutionDifference() << std::endl;
            }

            p.interpolate();


            nav_msgs::Path path;
            path.header.frame_id = "/map";
            path.header.stamp = goal.header.stamp;
            const std::vector<ob::State*>& states = p.getStates();
            for(std::vector<ob::State*>::const_iterator it = states.begin(); it != states.end(); ++it) {
                ob::SE2StateSpace::StateType& state = *(*it)->as<ob::SE2StateSpace::StateType>();
                geometry_msgs::PoseStamped pose;

                //map_info->cell2pointSubPixel(state[0], state[1],
                //                             pose.pose.position.x, pose.pose.position.y);
                pose.pose.position.x = state.getX();
                pose.pose.position.y = state.getY();
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(state.getYaw());

                path.poses.push_back(pose);
            }

            raw_path_publisher.publish(path);
            path_publisher.publish(path);
        }
        else {
            ROS_WARN("no path found");
        }
    }

    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
    {
        return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    }

private:
    ompl::app::KinematicCarPlanning setup;
    int maxWidth_;
    int maxHeight_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");

    OMPLPlanner planner;

    ros::WallRate r(30);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}
