
/// SYSTEM
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/PPM.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/config.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <omplapp/apps/KinematicCarPlanning.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <boost/filesystem.hpp>
#include <iostream>


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <utils_path/common/SimpleGridMap2d.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace ompl;


struct OMPLPlanner
{
    OMPLPlanner()
        : nh("~"), map_info(NULL)
    {
        std::string target_topic = "/move_base_simple/goal";
        nh.param("target_topic", target_topic, target_topic);

        goal_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                (target_topic, 2, boost::bind(&OMPLPlanner::updateGoal, this, _1));

        nh.param("use_map_topic", use_map_topic_, false);
        use_map_service_ = !use_map_topic_;

        if(use_map_topic_) {
            std::string map_topic = "/map/inflated";
            nh.param("map_topic",map_topic, map_topic);
            map_sub = nh.subscribe<nav_msgs::OccupancyGrid>
                    (map_topic, 1, boost::bind(&OMPLPlanner::updateMapCallback, this, _1));

        } else {
            map_service_client = nh.serviceClient<nav_msgs::GetMap>
                    ("/dynamic_map/inflated");
        }

        base_frame_ = "/base_link";
        nh.param("base_frame", base_frame_, base_frame_);


        path_publisher = nh.advertise<nav_msgs::Path> ("/path", 10);
        raw_path_publisher = nh.advertise<nav_msgs::Path> ("/path_raw", 10);



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

    void updateMapCallback (const nav_msgs::OccupancyGridConstPtr &map) {
        updateMap(*map);
    }


    void updateMap (const nav_msgs::OccupancyGrid &map) {
        unsigned w = map.info.width;
        unsigned h = map.info.height;

        bool replace = map_info == NULL ||
                map_info->getWidth() != w ||
                map_info->getHeight() != h;

        if(replace){
            if(map_info != NULL) {
                delete map_info;
            }
            map_info = new lib_path::SimpleGridMap2d(map.info.width, map.info.height, map.info.resolution);
        }

        /// Map data
        /// -1: unknown -> 0
        /// 0:100 probabilities -> 1 - 101
        std::vector<uint8_t> data(w*h);
        int i = 0;
        for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
            data[i++] = *it;
        }

        map_info->set(data, w, h);
        map_info->setOrigin(lib_path::Point2d(map.info.origin.position.x, map.info.origin.position.y));
        map_info->setLowerThreshold(10);
        map_info->setUpperThreshold(70);
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

    ompl::app::KinematicCarPlanning setup;
    int maxWidth_;
    int maxHeight_;

    void updateGoal (const geometry_msgs::PoseStampedConstPtr &goal) {
        ROS_INFO("got goal");

        if(use_map_service_) {
            nav_msgs::GetMap map_service;
            if(map_service_client.call(map_service)) {
                updateMap(map_service.response.map);
            }
        }
        lib_path::Pose2d from_world;
        lib_path::Pose2d to_world;

        tf::StampedTransform trafo;
        tfl.lookupTransform("/map", base_frame_, ros::Time(0), trafo);

        from_world.x = trafo.getOrigin().x();
        from_world.y = trafo.getOrigin().y();
        from_world.theta = tf::getYaw(trafo.getRotation());

        ROS_WARN_STREAM("theta=" << from_world.theta);

        to_world.x = goal->pose.position.x;
        to_world.y = goal->pose.position.y;
        to_world.theta = tf::getYaw(goal->pose.orientation);


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
            path.header.stamp = goal->header.stamp;
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

    lib_path::SimpleGridMap2d* map_info;

    ros::NodeHandle nh;

    ros::Subscriber goal_pose_sub;

    ros::Subscriber map_sub;
    ros::ServiceClient map_service_client;

    ros::Publisher path_publisher;
    ros::Publisher raw_path_publisher;

    bool use_map_topic_;
    bool use_map_service_;

    tf::TransformListener tfl;
    std::string base_frame_;
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
