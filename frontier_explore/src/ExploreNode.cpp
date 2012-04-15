/*
 * ExploreNode.cpp
 *
 * Created: Jan 2012
 * Author: Marks
 *
 */

// ROS
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>

// Project
#include "ExploreNode.h"

using namespace frontier_explore;
using namespace actionlib;

///////////////////////////////////////////////////////////////////////////////
// class ExploreNode
///////////////////////////////////////////////////////////////////////////////

ExploreNode::ExploreNode( ros::NodeHandle n ) :
    SimpleActionServer<ExplorationGoalsAction>( "exploration_goals",
                                                boost::bind( &ExploreNode::executeCB, this ),
                                                false ),
    tf_( ros::Duration( 5.0 )),
    goals_marker_count_( 0 )
{
    // Map options
    n.param( "use_ceiling_map", use_ceiling_map_, false );
    std::string ground_map_topic, ceiling_map_topic;
    n.param<std::string>( "ceiling_map_topic", ceiling_map_topic, "/ceiling_map" );
    n.param<std::string>( "ground_map_topic", ground_map_topic, "/map_inflated" );

    // Goal selection config
    double length_gain, ori_gain, path_gain, min_length;
    n.param<double>( "frontier_length_gain", length_gain, 1.0 );
    n.param<double>( "frontier_dist_gain", path_gain, 2.0 );
    n.param<double>( "frontier_orientation_gain", ori_gain, 5.0 );
    n.param<double>( "min_frontier_length", min_length, 0.5 );

    // Misc options
    n.param<bool>( "visualize", visualize_, false );

    // Setup
    explorer_.setFrontierLengthGain( length_gain );
    explorer_.setOrientationChangeGain( ori_gain );
    explorer_.setPathLengthGain( path_gain );
    explorer_.setMinFrontierLength( min_length );

    // Neccessary ROS stuff
    ground_map_subs_ = n.subscribe<nav_msgs::OccupancyGrid>( ground_map_topic, 0, boost::bind( &ExploreNode::groundMapCB, this, _1 ));
    if ( use_ceiling_map_ )
        ceiling_map_subs_ = n.subscribe<nav_msgs::OccupancyGrid>( ceiling_map_topic, 0, boost::bind( &ExploreNode::ceilingMapCB, this, _1 ));
    if ( visualize_ )
        visu_pub_ = n.advertise<visualization_msgs::Marker>( "visualization_markers", 100 );

    // Start action server
    start();
}

void ExploreNode::executeCB() {
    ExplorationGoalsResult result;

    // Check if we got valid maps
    if ( !got_ground_map_ ) {
        ROS_WARN( "Exploration goals requested but there is no ground (or ceiling) map yet. Wait until the data is published." );
        result.status = ExplorationGoalsResult::NO_MAP;
        setAborted( result );
        return;
    }

    // Preempt?
    if ( isPreemptRequested() || !ros::ok()) {
        result.status = ExplorationGoalsResult::PREEMPTED;
        setPreempted( result );
        return;
    }

    // Get the robot position
    Eigen::Vector3d robot_pose;
    if ( !getRobotPose( robot_pose, "/map" /*ground_map_data_.header.frame_id*/ )) {
        ROS_ERROR( "Cannot get robot position." );
        result.status = ExplorationGoalsResult::NO_POSE;
        setAborted( result );
        return;
    }

    // Caluclate the frontiers
    lib_ros_util::OccupancyGridWrapper ground_map( &ground_map_data_ );
    std::vector<WeightedFrontier> exploration_goals;
    if ( use_ceiling_map_ && got_ceiling_map_ ) {
        // Combine the ceiling and the ground map
        lib_ros_util::OccupancyGridWrapper ceiling_map( &ceiling_map_data_ );
        map_gen_.update( ground_map, ceiling_map );
        explorer_.getExplorationGoals( map_gen_.map, ground_map,robot_pose, exploration_goals );
    } else {
        // Use the ground map as exploration map
        explorer_.getExplorationGoals( ground_map, ground_map, robot_pose, exploration_goals );
    }

    // Visualize detected frontiers?
    if ( visualize_ )
        publishGoalsVisu( exploration_goals );

    // Copy goals
    geometry_msgs::Pose goal_pose;
    Eigen::Vector3d frontier_pose;
    for ( size_t i = 0; i < exploration_goals.size(); ++i ) {
        frontier_pose = exploration_goals[i].frontier.pose;
        goal_pose.position.x = frontier_pose.x();
        goal_pose.position.y = frontier_pose.y();
        goal_pose.position.z = 0;
        goal_pose.orientation = tf::createQuaternionMsgFromYaw( frontier_pose(2));
        result.goals.push_back( goal_pose );
    }

    // Return action result
    if ( result.goals.size() > 0 )
        result.status = ExplorationGoalsResult::OK;
    else
        result.status = ExplorationGoalsResult::DONE;
    setSucceeded( result );

    return;
}

bool ExploreNode::getRobotPose( Eigen::Vector3d &pose, const std::string& map_frame )
{
    tf::StampedTransform trafo;
    geometry_msgs::TransformStamped msg;
    try {
        tf_.lookupTransform( map_frame.c_str(), "/base_link", ros::Time(0), trafo );
    } catch (tf::TransformException& ex) {
        return false;
    }

    tf::transformStampedTFToMsg( trafo, msg );
    pose.x() = msg.transform.translation.x;
    pose.y() = msg.transform.translation.y;
    pose(2) = tf::getYaw( msg.transform.rotation );
    return true;
}

void ExploreNode::groundMapCB( const nav_msgs::OccupancyGridConstPtr &map )
{
    ground_map_data_ = *map;
    got_ground_map_ = true;
}

void ExploreNode::ceilingMapCB( const nav_msgs::OccupancyGridConstPtr &map )
{
    ceiling_map_data_ = *map;
    got_ceiling_map_ = true;
}

void ExploreNode::publishGoalsVisu(const std::vector<WeightedFrontier> &goals )
{
    visualization_msgs::Marker m;
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.ns = "exploration_goals";
    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 1.5;
    m.scale.y = 1.5;
    m.scale.z = 1.5;
    m.color.r = 255;
    m.color.g = 0;
    m.color.b = 0;
    m.color.a = 200;
    m.lifetime = ros::Duration(0);

    m.action = visualization_msgs::Marker::ADD;
    uint id = 0;
    unsigned int count = 0;
    Eigen::Vector3d p;
    for ( std::size_t i = 0; i < goals.size(); ++i ) {
        m.id = id;
        p = goals[i].frontier.pose;
        m.pose.position.x = p.x();
        m.pose.position.y = p.y();
        m.pose.position.z = 0;
        m.pose.orientation = tf::createQuaternionMsgFromYaw( p(2));
        visu_pub_.publish( m );
        m.color.r = 0; // Only the first goal red
        m.color.b = 255;
        ++id;
        ++count;
    }

    m.action = visualization_msgs::Marker::DELETE;
    for (; id < goals_marker_count_; id++) {
        m.id = id;
        visu_pub_.publish( m );
    }
    goals_marker_count_ = count;
}

int main( int argc, char* argv[] ) {

    ros::init(argc,argv, "frontier_explore");
    ros::NodeHandle n("~");
    ExploreNode explore_node( n );
    ros::spin();

    return 0;
}
