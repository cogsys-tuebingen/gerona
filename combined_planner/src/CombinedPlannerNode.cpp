/**
 * @file CombinedPlannerNode.cpp
 * @date Jan 2012
 * @author marks
 */

// ROS
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <visualization_msgs/Marker.h>

// Workspace
#include <utils/LibUtil/Stopwatch.h>

// Project
#include "CombinedPlannerNode.h"


using namespace lib_path;

///////////////////////////////////////////////////////////////////////////////
// class CombinedPlannerNode
///////////////////////////////////////////////////////////////////////////////


CombinedPlannerNode::CombinedPlannerNode()
    : n_( "~" ),
      tf_( ros::Duration( 5.0 )),
      map_( NULL ),
      a_star_( NULL ),
      got_map_( false )
{
    // Topic names parameters
    n_.param<std::string>( "map_topic", map_topic_, "/map_inflated" );
    n_.param<std::string>( "goal_topic", goal_topic_, "/goal" );
    n_.param<std::string>( "path_topic", path_topic_, "/a_star_path" );

    // Subscribe
    map_subs_ = n_.subscribe<nav_msgs::OccupancyGrid>( map_topic_, 1, boost::bind( &CombinedPlannerNode::updateMap, this, _1 ));
    goal_subs_ = n_.subscribe<geometry_msgs::PoseStamped>( goal_topic_, 1, boost::bind( &CombinedPlannerNode::updateGoal, this, _1 ));

    // Advertise
    path_pub_ = n_.advertise<nav_msgs::Path>( path_topic_, 5 );
    visu_pub_ = n_.advertise<visualization_msgs::Marker>( "visualization_markers", 5 );
}

void CombinedPlannerNode::updateMap( const nav_msgs::OccupancyGridConstPtr &map )
{
    // Create map?
    if ( map_ == NULL ) {
        map_ = new SimpleGridMap2d( map->info.width, map->info.height, map->info.resolution );
        ROS_INFO( "Received first map: Width %d, height %d, resolution %f",
                  map->info.width, map->info.height, map->info.resolution );
    }

    // Map setup
    map_->set( map->data, map->info.width, map->info.height );
    map_->setOrigin( Point2d( map->info.origin.position.x, map->info.origin.position.x ));
    map_frame_id_ = "/map"; //map->header.frame_id;

    // Create A* planner on first map update
    if ( a_star_ == NULL ) {
        a_star_ = new AStar( map_ );
    }

    // Ready to go
    got_map_ = true;
}

void CombinedPlannerNode::updateGoal( const geometry_msgs::PoseStampedConstPtr &goal )
{
    if ( !got_map_ ) {
        ROS_WARN( "Got a goal but no map. It's not possible to plan a path!" );
        return;
    }

    // Convert goal int map coordinate system if necessary
    geometry_msgs::PoseStamped goalMap;
    if ( goal->header.frame_id.compare( map_frame_id_ ) != 0 ) {
        try {
            tf_.transformPose( map_frame_id_, *goal, goalMap );
        } catch (tf::TransformException& ex) {
            ROS_ERROR( "Cannot transform goal into map coordinates. Reason: %s",
                       ex.what());
            return;
        }
    } else {
        goalMap = *goal;
    }

    // Get the current robot pose in map coordinates
    geometry_msgs::Pose robot_pose;
    if (!getRobotPose( robot_pose, map_frame_id_ ))
        return;

    // A* search
    Stopwatch stopwatch;
    a_star_->setNewMap( map_ );
    waypoint_t start, end;
    map_->point2Cell( Point2d( robot_pose.position.x, robot_pose.position.y), start.x, start.y );
    map_->point2Cell( Point2d( goalMap.pose.position.x, goalMap.pose.position.y), end.x, end.y );
    bool global_path_found = a_star_->planPath( start, end );
    ROS_INFO( "A* search took %d ms.", stopwatch.msElapsed());
    if ( !global_path_found ) {
        ROS_ERROR( "No global path found." );
        return;
    }

    // Waypoints from cell coordinates to map coordinates
    std::vector<geometry_msgs::Point> global_path;
    geometry_msgs::Point p;
    lib_path::Point2d g;
    lib_path::path_t* cell_path = a_star_->getLastPath();
    for ( std::size_t i = 0; i < cell_path->size(); ++i ) {
        map_->cell2point( (*cell_path)[i].x, (*cell_path)[i].y, g );
        p.x = g.x;
        p.y = g.y;
        p.z = 0;
        global_path.push_back( p );
    }

    ROS_INFO( "Global path found. Flattening path..." );

    std::size_t startIdx = 0;
    std::size_t endIdx = 1;
    global_path.clear();
    map_->cell2point( (*cell_path)[0].x, (*cell_path)[0].y, g );
    p.x = g.x;
    p.y = g.y;
    p.z = 0;
    global_path.push_back( p );
    while ( endIdx < cell_path->size() - 1) {
        if ( isFree( (*cell_path)[startIdx], (*cell_path)[endIdx]) ) {
            ++endIdx;
        } else {
            map_->cell2point( (*cell_path)[endIdx].x, (*cell_path)[endIdx].y, g );
            p.x = g.x;
            p.y = g.y;
            p.z = 0;
            global_path.push_back( p );
            startIdx = endIdx;
            endIdx++;
        }
    }
    endIdx = cell_path->size() - 1;
    map_->cell2point( (*cell_path)[endIdx].x, (*cell_path)[endIdx].y, g );
    p.x = g.x;
    p.y = g.y;
    p.z = 0;
    global_path.push_back( p );

    ROS_INFO( "Robot pose is: %f %f", robot_pose.position.x, robot_pose.position.y );
    for ( std::size_t i = 0; i < global_path.size(); ++i ) {
        ROS_INFO("Waypoint: %f %f", global_path[i].x, global_path[i].y );
    }
    ROS_INFO( "Goal is: %f %f", goalMap.pose.position.x, goalMap.pose.position.y );

    ROS_INFO( "Global path found. Publishing visualization..." );
    visualizeGlobalPath( global_path );
}

bool CombinedPlannerNode::getRobotPose( geometry_msgs::Pose& pose, const std::string& map_frame ) {
    tf::StampedTransform trafo;
    geometry_msgs::TransformStamped msg;
    try {
        tf_.lookupTransform( map_frame.c_str(), "/base_link", ros::Time(0), trafo );
    } catch (tf::TransformException& ex) {
        ROS_ERROR( "Error getting the robot position. Reason: %s",
                   ex.what());
        return false;
    }

    tf::transformStampedTFToMsg( trafo, msg );
    pose.position.x = msg.transform.translation.x;
    pose.position.y = msg.transform.translation.y;
    pose.position.z = 0;
    pose.orientation = msg.transform.rotation;

    return true;
}

bool CombinedPlannerNode::isFree( const lib_path::waypoint_t p1, const lib_path::waypoint_t p2 )
{
    bool free = true;

    if ( p1.x != p2.x || p1.y != p2.y) {
        // Traverse from left to right
        int x1, y1, x2, y2;
        if (p1.x <= p2.x) {
            x1 = p1.x;
            y1 = p1.y;
            x2 = p2.x;
            y2 = p2.y;
        } else {
            x1 = p2.x;
            y1 = p2.y;
            x2 = p1.x;
            y2 = p1.y;
        }

        int stepX = 1;
        int stepY = (y1 <= y2 ? 1 : -1);

        double tDeltaX = 0.0;
        double tDeltaY = 0.0;

        if (x1 == x2) {
            tDeltaX = 0.0;
            tDeltaY = 1.0;
        } else if (y1 == y2) {
            tDeltaX = 1.0;
            tDeltaY = 0.0;
        } else {
            double m = double(y2 - y1) / double(x2 - x1);
            tDeltaX = sqrt(1.0 + m * m);
            tDeltaY = sqrt(1.0 + 1.0 / (m * m));
        }

        double tMaxX = tDeltaX * 0.5;
        double tMaxY = tDeltaY * 0.5;

        if ( map_ != NULL ) {
            if ( map_->getWidth() > x2 && map_->getHeight() > max(y1, y2)) {
                int x = x1;
                int y = y1;

                while (x <= x2 && (stepY == 1 ? y <= y2 : y >= y2)) {
                    if ( map_->isOccupied( x, y )) {
                        free = false;
                        break;
                    }

                    if (tMaxX < tMaxY) {
                        tMaxX += tDeltaX;
                        x += stepX;
                    } else {
                        tMaxY += tDeltaY;
                        y += stepY;
                    }
                }
            } else {
                free = false;
            }
        } else {
            free = false;
        }
    }
    return free;
}

void CombinedPlannerNode::visualizeGlobalPath( const std::vector<geometry_msgs::Point> &path )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "global_path";
    marker.color.r = 1.0f;
    marker.color.a = 0.8f;
    marker.scale.x = marker.scale.y = 0.2f;
    marker.id = 0;
    for ( std::size_t i = 0; i < path.size(); ++i ) {
        marker.points.push_back( path[i] );
    }
    visu_pub_.publish( marker );
}


///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "combined_planner" );
    CombinedPlannerNode node;

    ros::Rate rate( 100 );
    while ( ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
