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

///////////////////////////////////////////////////////////////////////////////
// class ExploreNode
///////////////////////////////////////////////////////////////////////////////

using namespace actionlib;

namespace frontier_explore {

ExploreNode::ExploreNode( ros::NodeHandle n ) :
    SimpleActionServer<ExplorationGoalsAction>( "exploration_goals",
                                                boost::bind( &ExploreNode::executeCB, this ),
                                                false ),
    tf_( ros::Duration( 5.0 )),
    goals_marker_count_( 0 ),
    got_ground_map_( false ),
    got_ceiling_map_( false )
{
    // Map options
    n.param( "use_ceiling_map", use_ceiling_map_, false );
    std::string ground_map_topic, ceiling_map_topic;
    n.param<std::string>( "ceiling_map_topic", ceiling_map_topic, "/map_ceiling" );
    n.param<std::string>( "ground_map_topic", ground_map_topic, "/map_inflated" );

    // Goal selection config
    double importance_gain, ori_gain, path_gain, min_length;
    n.param<double>( "importance_gain", importance_gain, 1.0 );
    n.param<double>( "dist_gain", path_gain, 2.0 );
    n.param<double>( "orientation_gain", ori_gain, 5.0 );
    n.param<double>( "min_frontier_length", min_length, 0.5 );

    // Misc options
    n.param<bool>( "visualize", visualize_, false );

    // Setup
    explorer_.setImportanceGain( importance_gain );
    explorer_.setOrientationChangeGain( ori_gain );
    explorer_.setPathLengthGain( path_gain );
    explorer_.setMinFrontierLength( min_length );

    // Neccessary ROS stuff
    ground_map_subs_ = n.subscribe<nav_msgs::OccupancyGrid>( ground_map_topic, 0, boost::bind( &ExploreNode::groundMapCB, this, _1 ));
    if ( use_ceiling_map_ )
        ceiling_map_subs_ = n.subscribe<nav_msgs::OccupancyGrid>( ceiling_map_topic, 0, boost::bind( &ExploreNode::ceilingMapCB, this, _1 ));
    if ( visualize_ )
        visu_pub_ = n.advertise<visualization_msgs::Marker>( "visualization_markers", 10 );

    // Start action server
    start();
}

void ExploreNode::executeCB() {
    ExplorationGoalsResult result;
    ExplorationGoalsGoal action_goal; /// @todo use the goal! How do I access it?

    // Check if we got valid maps
    if ( !got_ground_map_ ) {
        ROS_WARN( "Exploration goals requested but there is no ground map yet. Wait until the data is published." );
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

    // Transform given points of interest
    std::vector<frontier_explore::Goal> poi;
    for ( size_t i = 0; i < action_goal.poi.size(); ++i ) {
        frontier_explore::Goal poi_goal;
        if ( !poseStampedToInternal( action_goal.poi[i], poi_goal.pose )) {
            result.status = ExplorationGoalsResult::NO_TRANSFORM;
            setAborted( result );
            return;
        }
        poi_goal.importance = action_goal.importance[i];
        poi.push_back( poi_goal );
    }

    // Caluclate the frontiers
    lib_ros_util::OccupancyGridWrapper ground_map( &ground_map_data_ );
    if ( use_ceiling_map_ && got_ceiling_map_ ) {
        // Combine the ceiling and the ground map
        lib_ros_util::OccupancyGridWrapper ceiling_map( &ceiling_map_data_ );
        map_gen_.update( ground_map, ceiling_map );
        explorer_.calculateGoals( &map_gen_.map, &ground_map, robot_pose, poi );
    } else {
        // Use the ground map as exploration map
        explorer_.calculateGoals( &ground_map, &ground_map, robot_pose, poi );
    }

    // Get the goals
    std::vector<WeightedGoal> goals;
    explorer_.getGoals( goals );

    // Visualize detected frontiers?
    if ( visualize_ )
        publishGoalsVisu( goals );

    // Copy goals
    geometry_msgs::PoseStamped goal_pose;
    ros::Time now = ros::Time::now();
    Eigen::Vector3d p;
    for ( size_t i = 0; i < goals.size(); ++i ) {
        p = goals[i].goal.pose;
        goal_pose.pose.position.x = p.x();
        goal_pose.pose.position.y = p.y();
        goal_pose.pose.position.z = 0;
        goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw( p(2));
        goal_pose.header.frame_id = "/map";
        goal_pose.header.stamp = now;
        result.goals.push_back( goal_pose );
        result.distance.push_back( goals[i].dist );
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

void ExploreNode::publishGoalsVisu(const std::vector<WeightedGoal> &goals )
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
        p = goals[i].goal.pose;
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

bool ExploreNode::poseStampedToInternal( const geometry_msgs::PoseStamped &in,
                                         Eigen::Vector3d &out )
{
    geometry_msgs::PoseStamped out_stamped;
    try {
        tf_.waitForTransform( "/map", in.header.frame_id, ros::Time(0), ros::Duration( 1.0 ));
        tf_.transformPose( "/map", in, out_stamped );
    } catch ( tf::TransformException &ex ) {
        ROS_ERROR( "Cannot transform pose. Reason: %s", ex.what());
        return false;
    }

    out.x() = out_stamped.pose.position.x;
    out.y() = out_stamped.pose.position.y;
    out(2) = tf::getYaw( out_stamped.pose.orientation );

    return true;
}

} // namespace

int main( int argc, char* argv[] ) {

    ros::init(argc,argv, "frontier_explore");
    ros::NodeHandle n("~");
    frontier_explore::ExploreNode explore_node( n );
    ros::spin();

    return 0;
}
