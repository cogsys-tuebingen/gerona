#include "pathfollower.h"
#include <cmath>

#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include "exceptions.h"
#include "mapprocessor.h"

using namespace traversable_path;
using namespace Eigen;

PathFollower::PathFollower() :
    motion_control_action_client_("motion_control"),
    current_goal_(0,0),
    path_angle_(NAN)
{
    subscribe_map_ = node_handle_.subscribe("traversability_map", 0, &PathFollower::mapCallback, this);
    publish_rviz_marker_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    publish_goal_ = node_handle_.advertise<geometry_msgs::PoseStamped>("traversable_path/goal", 1);
}


void PathFollower::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    map_ = msg;

    try {
        /** \todo extra method for all the refreshes */
        if (refreshRobotPose()) {
            refreshPathLine();
            refreshPathDirectionAngle();

            // distance of robot to path middle line
            Vector2f to_mid_line = vectorFromPointToLine(path_middle_line_, robot_pose_.position);
            // goal position (1m ahead):
            Vector2f goal_pos = robot_pose_.position + 0.8 * path_middle_line_.direction
                    + to_mid_line;


            // check if goalpoint is traversable (and reachable) at all.

            // convert points to pixel coordinates of the map.
            Vector2f robot_on_map = transformToMap(robot_pose_.position);
            Vector2f goal_on_map = transformToMap(goal_pos);

            bool noObstacle = MapProcessor::checkTraversabilityOfLine(*map_,
                                                                      cv::Point2i(robot_on_map[0], robot_on_map[1]),
                                                                      cv::Point2i(goal_on_map[0], goal_on_map[1]));
            if (noObstacle) {
                setGoalPoint(goal_pos, path_angle_);
            }
            else {
                ROS_INFO("OBSTACLE AHEAD! Stop moving.");
                /** \todo is this the stop commend? Ask Hendrik or Karsten */
                motion_control_action_client_.cancelGoal();

                /** \todo Handle this. Don't just stop. */
            }
        }
    }
    catch (const TransformMapException &e) {
        ROS_WARN_THROTTLE(1, "%s", e.what());
    }
    catch (const Exception &e) {
        ROS_WARN_THROTTLE(1, "Unknown path direction: %s", e.what());
    }
}


void PathFollower::publishGoalMarker(Vector2f position, float theta) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.pose.position = vectorToPoint(position);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "follow_path";
    marker.id = 0;

    // Set the marker type.
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker
    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    // Publish the marker
    publish_rviz_marker_.publish(marker);
}

void PathFollower::publishLineMarker(Eigen::Vector2f p1, Eigen::Vector2f p2, int id, std_msgs::ColorRGBA color) const
{
    visualization_msgs::Marker line;

    line.header.frame_id = "/map";
    line.ns = "follow_path/lines";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.id = id;
    line.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line.scale.x = 0.03;
    line.color = color;

    // set the points
    line.points.push_back(vectorToPoint(p1));
    line.points.push_back(vectorToPoint(p2));

    publish_rviz_marker_.publish(line);
}

void PathFollower::publishArrowMarker(Eigen::Vector2f point, float angle, int id, std_msgs::ColorRGBA color) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.pose.position = vectorToPoint(point);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    marker.ns = "follow_path/arrow";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color = color;

    publish_rviz_marker_.publish(marker);
}

void PathFollower::publishArrowMarker(Eigen::Vector2f point, Eigen::Vector2f direction, int id, std_msgs::ColorRGBA color) const
{
    float angle = atan2(direction[1], direction[0]);
    publishArrowMarker(point, angle, id, color);
}

void PathFollower::setGoalPoint(Vector2f position, float theta)
{
    const float MIN_DISTANCE_BETWEEN_GOALS = 0.5;

    // make sure the min. distance doesn't avoid the goal to be set at the first call of this method.
    float distance = INFINITY;
    if (!current_goal_.isZero()) {
        distance = (position - current_goal_).norm();
    }

    if (distance > MIN_DISTANCE_BETWEEN_GOALS) {
        // send goal to motion_control
        motion_control::MotionGoal goal;
        goal.v     = 0.4;
        goal.beta  = 0;
        //goal.pos_tolerance = 0.1;
        goal.mode  = motion_control::MotionGoal::MOTION_TO_GOAL;

        goal.x     = position[0];
        goal.y     = position[1];
        goal.theta = theta;

        // send goal to motion_control
        motion_control_action_client_.sendGoal(goal);
        // set as current goal
        current_goal_ = position;
        // send goal-marker to rviz for debugging
        publishGoalMarker(position, path_angle_);

//        ROS_DEBUG("Goal (map): x: %f; y: %f; theta: %f;; x: %f, y: %f, z: %f, w: %f", goal_point_map.pose.position.x,
//                  goal_point_map.pose.position.y, tf::getYaw(goal_point_map.pose.orientation),
//                  goal_point_map.pose.orientation.x,
//                  goal_point_map.pose.orientation.y,
//                  goal_point_map.pose.orientation.z,
//                  goal_point_map.pose.orientation.w);
    }
    else {
        ROS_DEBUG_THROTTLE(0.5, "Didn't update goal. New goal is %.2f m distant from the current goal. Minimum distance is %f",
                  distance, MIN_DISTANCE_BETWEEN_GOALS);
    }
}

bool PathFollower::refreshRobotPose()
{
    try {
        // position/orientation of the robot
        tf::StampedTransform robot_pose;
        tf_listener_.lookupTransform("/map", "/base_link", ros::Time(0), robot_pose);
        // position
        robot_pose_.position[0] = robot_pose.getOrigin().getX();
        robot_pose_.position[1] = robot_pose.getOrigin().getY();
        // orientation
        btVector3 tmp(1,0,0);
        tmp = tmp.rotate(robot_pose.getRotation().getAxis(), robot_pose.getRotation().getAngle());
        robot_pose_.orientation[0] = tmp.getX();
        robot_pose_.orientation[1] = tmp.getY();
    }
    catch (tf::TransformException e) {
        ROS_WARN_THROTTLE_NAMED(1, "tf", "tf::TransformException in %s (line %d):\n%s", __FILE__, __LINE__, e.what());
        return false;
    }
    return true;
}

void PathFollower::refreshPathLine()
{
    vectorVector2f points_middle;
    findPathMiddlePoints(&points_middle);

    // need at least two points, otherwise the regression will fail.
    if (points_middle.size() < 2) {
        throw Exception("Missing path points");
    }

    // fit a line to the points
    Line new_line;
    fitLinear(points_middle, &new_line);

    // make sure the direction vector of the line points in the direction that is nearer to the robot orientation.
    float angle = acos( new_line.direction.dot(robot_pose_.orientation)
                        / (new_line.direction.norm() * robot_pose_.orientation.norm()) );
    // if angle is greater than 90°, invert direction.
    if (angle > M_PI/2) {
        new_line.direction *= -1;
    }

    // filter line
    if (path_middle_line_.direction.isZero()) {
        path_middle_line_ = new_line;
        ROS_DEBUG_STREAM("direction unfiltered: " << path_middle_line_.direction);
    } else {

        path_middle_line_.point = 0.9*path_middle_line_.point + 0.1*new_line.point;
        path_middle_line_.direction = 0.9*path_middle_line_.direction + 0.1*new_line.direction;
        path_middle_line_.direction.normalize();
        //path_middle_line_.normal = 0.9*path_middle_line_.normal + 0.1*new_line.normal;
        path_middle_line_.normal[0] = - path_middle_line_.direction[1];
        path_middle_line_.normal[1] = path_middle_line_.direction[0];

        ROS_DEBUG_STREAM("direction: " << new_line.direction);
        ROS_DEBUG_STREAM("direction filtered: " << path_middle_line_.direction);
    }

    /////////////// MARKER
    {
        // points
        visualization_msgs::Marker points;
        points.header.frame_id = "/map";
        points.ns = "follow_path/mid_points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 1;
        points.type = visualization_msgs::Marker::POINTS;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        // Points are cyan
        points.color.g = 1.0;
        points.color.b = 1.0;
        points.color.a = 1.0;
        for (vectorVector2f::iterator it = points_middle.begin(); it != points_middle.end(); ++it) {
            geometry_msgs::Point p;
            p.x = (*it)[0];
            p.y = (*it)[1];
            p.z = 0;
            points.points.push_back(p);
        }
        publish_rviz_marker_.publish(points);

        // line
        std_msgs::ColorRGBA color;
        color.r = 1.0; color.g = 0.5; color.a = 1.0; // orange
        Vector2f p1, p2;
        p1 = path_middle_line_.point + 5 * path_middle_line_.direction;
        p2 = path_middle_line_.point - 5 * path_middle_line_.direction;
        publishLineMarker(p1, p2, 13, color);

        // line
        color.r = 1.0; color.g = 0.0; color.a = 1.0; //red
        p1 = new_line.point + 3 * new_line.direction;
        p2 = new_line.point - 3 * new_line.direction;
        publishLineMarker(p1, p2, 14, color);
    }
}

void PathFollower::refreshPathDirectionAngle()
{
    //// calculate angle
    // angle of the path
    float theta = atan2(path_middle_line_.direction[1], path_middle_line_.direction[0]);

    // make sure we follow the path in the right direction (if angle of robot to path is > 90°, change direction about
    // 180°).
//    float angle_robot = atan2(robot_pose_.orientation[1], robot_pose_.orientation[0]);
//    float angle_to_path = angle_robot - theta;
//    if (fabs(angle_to_path) > M_PI/2) {
//        //ROS_BREAK(); // i think this should not happen any more.
//        // change directory angle about 180° and also revert direction vector of the line
//        theta = theta - M_PI;
//        path_middle_line_direction_signum_ = -1;
//    } else {
//        path_middle_line_direction_signum_ = +1;
//    }


    path_angle_ = theta;


    /////////////// MARKER
//    {
//        // direction arrow (red)
//        visualization_msgs::Marker direction_marker;
//        direction_marker.header.frame_id = "/map";
//        direction_marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
//        direction_marker.pose.position = vectorToPoint(robot_pose_.position);
//        direction_marker.ns = "follow_path/direction";
//        direction_marker.id = 1;
//        direction_marker.type = visualization_msgs::Marker::ARROW;
//        direction_marker.action = visualization_msgs::Marker::ADD;
//        direction_marker.scale.x = direction_marker.scale.y = direction_marker.scale.z = 0.8;
//        direction_marker.color.r = 1.0;
//        direction_marker.color.a = 1.0;
//        publish_rviz_marker_.publish(direction_marker);

//        // filtered direction arrow (green)
//        direction_marker.pose.orientation = tf::createQuaternionMsgFromYaw(path_angle_);
//        direction_marker.id = 2;
//        direction_marker.scale.x = direction_marker.scale.y = direction_marker.scale.z = 1.0;
//        direction_marker.color.g = 1.0;
//        direction_marker.color.r = 0.0;
//        publish_rviz_marker_.publish(direction_marker);
//        /////////////// END MARKER
//    }
}

bool PathFollower::findPathMiddlePoints(PathFollower::vectorVector2f *out) const
{
    ROS_ASSERT(out != 0);
    // Work in frame /map

    // Orthogonal vector of robot direction: (x,y) -> (-y,x)
    // Points left of the direction (should be the y-axis)
    Vector2f orthogonal;
    orthogonal[0] = - robot_pose_.orientation[1];
    orthogonal[1] = robot_pose_.orientation[0];


    /* *** get path middle points *** */
    //! Size of the steps when looking for the edge points.
    /** Using map resolution gives the greates possible step size which ensures that we miss no cell. */
    float step_size = map_->info.resolution;

    // go forward
    //! Position of the current forward step.
    Vector2f forward_pos = robot_pose_.position - robot_pose_.orientation;
    for (float forward = -1.0; forward < 1.0; forward += 3*step_size) {
        /** \todo better exception handling here? */
        forward_pos += robot_pose_.orientation * 3*step_size;

        // break, if obstacle is in front.
        try {
            if (map_->data[transformToMapIndex(forward_pos)] != 0) {
                continue;
            }
        } catch (TransformMapException e) {
            ROS_WARN_THROTTLE(1, "Ahead: %s", e.what());
            continue;
        }

        Vector2f left_edge = forward_pos, right_edge = forward_pos;
        try {
            // find left edge
            do {
                left_edge += orthogonal * step_size;
            }
            while( map_->data[transformToMapIndex(left_edge)] == 0 );

            // find right edge
            do {
                right_edge += -orthogonal * step_size;
            }
            while( map_->data[transformToMapIndex(right_edge)] == 0 );
        } catch (TransformMapException e) {
            //ROS_WARN("Cant find Edge: %s", e.what());
        }

        // middle of this points
        Vector2f middle_point = (left_edge + right_edge) / 2;
        out->push_back(middle_point);
    }

    // drop last point for it might disturb the line
    if (out->size()) {
        out->pop_back();
    }

    return true;
}

Vector2f PathFollower::transformToMap(Vector2f point) const
{
    Vector2f result;
    result[0] = (point[0] - map_->info.origin.position.x) / map_->info.resolution;
    result[1] = (point[1] - map_->info.origin.position.y) / map_->info.resolution;

    if (result[0] < 0 || result[0] >= (int)map_->info.width || result[1] < 0 || result[1] >= (int)map_->info.height) {
        throw TransformMapException();
    }

    return result;
}

size_t PathFollower::transformToMapIndex(Vector2f point) const
{
    Vector2f pixel = transformToMap(point);

    return pixel[1] * map_->info.width + pixel[0];
}

void PathFollower::fitLinear(const PathFollower::vectorVector2f &points, PathFollower::Line *result)
{
    /*
     * This is a modified version of the function fitHyperplane() of eigen3 (eigen2support/LeastSquares.h),
     * which seems to be deprecated and thus is not used here directly.
     */
    // need at least two points.
    ROS_ASSERT(points.size() > 1);
    ROS_ASSERT(result != 0);

    // compute the mean of the data
    Vector2f mean = Vector2f::Zero(2);
    for (size_t i = 0; i < points.size(); ++i) {
        mean += points[i];
    }
    mean /= points.size();

    // compute the covariance matrix
    Matrix2f covMat = Matrix2f::Zero(2,2);
    for (size_t i = 0; i < points.size(); ++i) {
        Vector2f diff = (points[i] - mean).conjugate();
        covMat += diff * diff.adjoint();
    }

    // now we just have to pick the eigen vector with largest eigen value
    SelfAdjointEigenSolver<Matrix2f> eig(covMat);
    result->normal    = eig.eigenvectors().col(0); // eigen vector with smallest eigen value (= normal)
    result->direction = eig.eigenvectors().col(1); // eigen vector with largest eigen value (= direction of the line)

    result->point = mean;
    result->soundness = eig.eigenvalues().coeff(0)/eig.eigenvalues().coeff(1);
}

Vector2f PathFollower::vectorFromPointToLine(const PathFollower::Line &line, const Vector2f &point) const
{
    float k = (point.dot(line.direction) - line.point.dot(line.direction)) / line.direction.dot(line.direction);
    Vector2f point_to_line = -(point - line.point - k*line.direction);

    /////////////// MARKER
    /*
    // arrow
    visualization_msgs::Marker direction_marker;
    direction_marker.header.frame_id = "/map";
    direction_marker.pose.orientation = tf::createQuaternionMsgFromYaw( atan2(point_to_line[1], point_to_line[0]) );
    direction_marker.pose.position.x = robot_pose_.position[0];
    direction_marker.pose.position.y = robot_pose_.position[1];
    direction_marker.pose.position.z = 0;
    direction_marker.ns = "follow_path/direction";
    direction_marker.id = 19;
    direction_marker.type = visualization_msgs::Marker::ARROW;
    direction_marker.action = visualization_msgs::Marker::ADD;
    direction_marker.scale.x = direction_marker.scale.y = direction_marker.scale.z = point_to_line.norm();
    direction_marker.color.b = 1.0;
    direction_marker.color.a = 1.0;
    publish_rviz_marker_.publish(direction_marker);
    /////////////////
    */

    return point_to_line;
}

geometry_msgs::Point PathFollower::vectorToPoint(Vector2f v)
{
    geometry_msgs::Point p;
    p.x = v[0];
    p.y = v[1];
    p.z = 0;

    return p;
}


//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_path");

    PathFollower follower;

    // main loop
    ros::spin();
    return 0;
}
