#include "pathfollower.h"

#include "exceptions.h"
#include <cmath>

using namespace traversable_path;
using namespace Eigen;

PathFollower::PathFollower() :
        motion_control_action_client_("motion_control"),
    path_angle_(NAN)
{
    subscribe_scan_classification_ = node_handle_.subscribe("path_classification_cloud", 10,
                                                            &PathFollower::scan_classification_callback, this);
    subscribe_map_ = node_handle_.subscribe("traversability_map", 0, &PathFollower::mapCallback, this);
    publish_rviz_marker_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    publish_goal_ = node_handle_.advertise<geometry_msgs::PoseStamped>("traversable_path/goal", 1);
}

void PathFollower::scan_classification_callback(const pcl::PointCloud<PointXYZRGBT>::ConstPtr &scan)
{
    int goal_index;

    // search traversable area in front of the robot (assuming, "in front" is approximalty in the middle of the scan)
    unsigned int mid = scan->points.size() / 2;

    // search traversable area
    unsigned int beginning = 0, end = 0;
    for (unsigned int i = 0; i < mid; ++i) {
        if (scan->points[mid+i].traversable) {
            beginning = mid+i;
            end = mid+i;
            break;
        } else if (scan->points[mid-i].traversable) {
            beginning = mid-i;
            end = mid-i;
            break;
        }
    }

    if (beginning == 0) {
        ROS_DEBUG("No traversable paths found.");
        /** @todo stop robot */
        return;
    }

    // get range of the area
    while (beginning > 0 && scan->points[beginning].traversable) {
        --beginning;
    }
    while (end < scan->points.size()-1 && scan->points[end].traversable) {
        ++end;
    }

    ROS_DEBUG("size points: %zu, beginning: %d, end: %d", scan->points.size(), beginning, end);

    // goal = point in the middle of the path
    goal_index = beginning + (end-beginning)/2;

    // transform goal-pose to map-frame
    geometry_msgs::PoseStamped goal_point_laser;
    geometry_msgs::PoseStamped goal_point_map;

    goal_point_laser.header = scan->header;
    goal_point_laser.pose.position.x = scan->points[goal_index].x;
    goal_point_laser.pose.position.y = scan->points[goal_index].y;
    goal_point_laser.pose.position.z = scan->points[goal_index].z;

    // orientation: orthogonal to the line of the traversable segment
    double delta_x = scan->points[end].x - scan->points[beginning].x;
    double delta_y = scan->points[end].y - scan->points[beginning].y;
    double theta = atan2(delta_y, delta_x);
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(theta), goal_point_laser.pose.orientation);
    ROS_DEBUG("B: %g, %g; E: %g, %g", scan->points[beginning].x, scan->points[beginning].y, scan->points[end].x, scan->points[end].y);
    ROS_DEBUG("dx = %f, dy = %f, atan2 = %f, theta = %f", delta_x, delta_y, atan2(delta_y, delta_x), theta);
    /** \todo wieder einkommentieren */
    //publishTraversaleLineMarker(scan->points[beginning], scan->points[end], scan->header);


    try {
        tf_listener_.transformPose("/map", goal_point_laser, goal_point_map);
        goal_point_map.pose.position.z = 0;
    }
    catch (tf::TransformException e) {
        ROS_WARN("Unable to transform goal. tf says: %s", e.what());
        return;
    }

    const double MIN_DISTANCE_BETWEEN_GOALS = 0.5;
    double distance = sqrt( pow(goal_point_map.pose.position.x - current_goal_.x, 2) +
                            pow(goal_point_map.pose.position.y - current_goal_.y, 2) );

    if (distance > MIN_DISTANCE_BETWEEN_GOALS) {
        // send goal to motion_control
        motion_control::MotionGoal goal;
        goal.v     = 0.4;
        goal.beta  = 0;
        //goal.pos_tolerance = 0.1;
        goal.mode  = motion_control::MotionGoal::MOTION_TO_GOAL;

        goal.x     = goal_point_map.pose.position.x;
        goal.y     = goal_point_map.pose.position.y;
        goal.theta = tf::getYaw(goal_point_map.pose.orientation);

        // send goal to motion_control
        motion_control_action_client_.sendGoal(goal);
        current_goal_ = goal_point_map.pose.position;

        // send goal-marker to rviz for debugging
        //publishGoalMarker(goal_point_map);

        ROS_DEBUG("Goal (map): x: %f; y: %f; theta: %f;; x: %f, y: %f, z: %f, w: %f", goal_point_map.pose.position.x,
                 goal_point_map.pose.position.y, tf::getYaw(goal_point_map.pose.orientation),
                 goal_point_map.pose.orientation.x,
                 goal_point_map.pose.orientation.y,
                 goal_point_map.pose.orientation.z,
                 goal_point_map.pose.orientation.w);
    }
    else {
        ROS_DEBUG("Didn't update goal. New goal is %.2f m distant from the current goal. Minimum distance is %f",
                  distance, MIN_DISTANCE_BETWEEN_GOALS);
    }
}


void PathFollower::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    map_ = msg;

    /** \todo only for testing */
    try {
        /** \todo extra method for all the refreshes */
        if (refreshRobotPose()) {
            refreshPathLine();
            refreshPathDirectionAngle();

            // distance of robot to path middle line
            float dist = distanceToLine(path_middle_line_, robot_pose_.position);
            // goal position (1m ahead):
            Vector2f goal_pos = robot_pose_.position + 1*path_middle_line_.direction + dist*path_middle_line_.normal;

            /** \todo pos buggt noch */

            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = "/map";
            goal.pose.position.x = goal_pos[0];
            goal.pose.position.y = goal_pos[1];
            goal.pose.orientation = tf::createQuaternionMsgFromYaw(path_angle_);
            publishGoalMarker(goal);
        }


    }
    catch (Exception e) {
        ROS_WARN("Unknown path direction: %s", e.what());
    }
}


void PathFollower::publishGoalMarker(const geometry_msgs::PoseStamped &goal) const
{
//    geometry_msgs::PoseStamped foo = goal;
//    foo.pose.orientation.x = 1;
//    foo.pose.orientation.y = 2;
//    foo.pose.orientation.z = 3;
//    foo.pose.orientation.w = 4;

    publish_goal_.publish(goal);

    visualization_msgs::Marker marker;

    marker.header = goal.header;
    marker.pose   = goal.pose;

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

void PathFollower::publishTraversaleLineMarker(PointXYZRGBT a, PointXYZRGBT b, std_msgs::Header header) const
{
    //ROS_INFO("mark line from point a(%f,%f,%f) to b(%f,%f,%f)", a.x, a.y, a.z, b.x, b.y, b.z);

    visualization_msgs::Marker points, line_strip;

    points.header = line_strip.header = header;
    points.ns = line_strip.ns = "follow_path";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 1;
    line_strip.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;
    points.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.03;

    // Points are blue
    points.color.b = 1.0;
    points.color.a = 1.0;

    // Line strip is green
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;


    // why the hell can't i simply cast this points??
    geometry_msgs::Point pa, pb;
    pa.x = a.x; pa.y = a.y; pa.z = a.z;
    pb.x = b.x; pb.y = b.y; pb.z = b.z;

    points.points.push_back(pa);
    points.points.push_back(pb);
    line_strip.points.push_back(pa);
    line_strip.points.push_back(pb);

    publish_rviz_marker_.publish(points);
    publish_rviz_marker_.publish(line_strip);
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
    geometry_msgs::Point p;
    p.x = p1[0];
    p.y = p1[1];
    line.points.push_back(p);
    p.x = p2[0];
    p.y = p2[1];
    line.points.push_back(p);

    publish_rviz_marker_.publish(line);
}

void PathFollower::publishLineMarker(Vector2f coefficients, int min_x, int max_x, int id, std_msgs::ColorRGBA color) const
{
    Vector2f p1, p2;

    // set the points
    p1[0] = min_x;
    p1[1] = coefficients[0]*min_x + coefficients[1];
    p2[0] = max_x;
    p2[1] = coefficients[0]*max_x + coefficients[1];

    publishLineMarker(p1, p2, id, color);
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
        ROS_WARN("tf::TransformException in %s (line %d):\n%s", __FILE__, __LINE__, e.what());
        return false;
    }
    return true;
}

void PathFollower::refreshPathLine()
{
    vectorVector2f points_middle;
    findPathMiddlePoints(&points_middle);

    if (points_middle.size() == 0) {
        throw Exception("Missing path points");
    }

    // fit a line to the points
    fitLinear(points_middle, &path_middle_line_);

    /////////////// MARKER
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
}

void PathFollower::refreshPathDirectionAngle()
{
    //// calculate angle
    // angle of the path
    float theta = atan2(path_middle_line_.direction[1], path_middle_line_.direction[0]);

    // make sure we follow the path in the right direction (if angle of robot to path is > 90°, change direction about
    // 180°).
    float angle_robot = atan2(robot_pose_.orientation[1], robot_pose_.orientation[0]);
    float angle_to_path = angle_robot - theta;
    if (fabs(angle_to_path) > M_PI/2) {
        theta = theta - M_PI;
    }


    //// Filter
    if (isnan(path_angle_)) {
        // first time initialization
        path_angle_ = theta;
    } else {
        //path_angle_ = 0.9*path_angle_ + 0.1*theta;

        // we have a problem here with the preiodic nature of angles, so it's a bit more complicated:
        // instead of a = 0.9a+0.1n use a = 0.1(n-a) (which is the same). Take care of overflows (+pi%2pi-pi for that).
        float delta = fmod(theta - path_angle_ + M_PI, 2*M_PI) - M_PI;
        path_angle_ += 0.1 * delta;
    }


    /////////////// MARKER
    // direction arrow (red)
    visualization_msgs::Marker direction_marker;
    direction_marker.header.frame_id = "/map";
    direction_marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    direction_marker.pose.position.x = robot_pose_.position[0];
    direction_marker.pose.position.y = robot_pose_.position[1];
    direction_marker.pose.position.z = 0;
    direction_marker.ns = "follow_path/direction";
    direction_marker.id = 1;
    direction_marker.type = visualization_msgs::Marker::ARROW;
    direction_marker.action = visualization_msgs::Marker::ADD;
    direction_marker.scale.x = direction_marker.scale.y = direction_marker.scale.z = 0.8;
    direction_marker.color.r = 1.0;
    direction_marker.color.a = 1.0;
    publish_rviz_marker_.publish(direction_marker);

    // filtered direction arrow (green)
    direction_marker.pose.orientation = tf::createQuaternionMsgFromYaw(path_angle_);
    direction_marker.id = 2;
    direction_marker.scale.x = direction_marker.scale.y = direction_marker.scale.z = 1.0;
    direction_marker.color.g = 1.0;
    direction_marker.color.r = 0.0;
    publish_rviz_marker_.publish(direction_marker);
    /////////////// END MARKER
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
            if (map_->data[transformToMap(forward_pos)] != 0) {
                continue;
            }
        } catch (TransformMapException e) {
            ROS_WARN("Ahead: %s", e.what());
            continue;
        }

        Vector2f left_edge = forward_pos, right_edge = forward_pos;
        try {
            // find left edge
            do {
                left_edge += orthogonal * step_size;
            }
            while( map_->data[transformToMap(left_edge)] == 0 );

            // find right edge
            do {
                right_edge += -orthogonal * step_size;
            }
            while( map_->data[transformToMap(right_edge)] == 0 );
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

size_t PathFollower::transformToMap(Vector2f point) const
{
    int x, y;
    x = (point[0] - map_->info.origin.position.x) / map_->info.resolution;
    y = (point[1] - map_->info.origin.position.y) / map_->info.resolution;

    if (x < 0 || x >= (int)map_->info.width || y < 0 || y >= (int)map_->info.height) {
        throw TransformMapException();
    }

    return y * map_->info.width + x;
}

void PathFollower::fitLinear(const PathFollower::vectorVector2f &points, PathFollower::Line *result)
{
    /*
     * This is a modified version of the function fitHyperplane() of eigen3 (eigen2support/LeastSquares.h),
     * which seems to be deprecated and thus is not used here directly.
     */
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

float PathFollower::distanceToLine(const PathFollower::Line &line, const Vector2f &point)
{
    float k = (point.dot(line.direction) - line.point.dot(line.direction)) / line.direction.dot(line.direction);
    return (point - line.point - k*line.direction).norm();
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
