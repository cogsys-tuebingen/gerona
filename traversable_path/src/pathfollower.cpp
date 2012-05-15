#include "pathfollower.h"

using namespace traversable_path;

PathFollower::PathFollower() :
        motion_control_action_client_("motion_control")
{
    subscribe_scan_classification_ = node_handle_.subscribe("path_classification_cloud", 100,
                                                            &PathFollower::scan_classification_callback, this);
    subscribe_map_ = node_handle_.subscribe("traversability_map", 0, &PathFollower::mapCallback, this);
    publish_rviz_marker_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
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
        publishGoalMarker(goal_point_map);

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
        getPathDirectionAngle();
    }
    catch (Exception e) {
        ROS_WARN("Unknown path direction: %s", e.what());
    }
}


void PathFollower::publishGoalMarker(const geometry_msgs::PoseStamped &goal)
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
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    publish_rviz_marker_.publish(marker);
}

void PathFollower::publishTraversaleLineMarker(PointXYZRGBT a, PointXYZRGBT b, std_msgs::Header header)
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

void PathFollower::publishLineMarker(Eigen::Vector2f coefficients, int min_x, int max_x, int id, std_msgs::ColorRGBA color)
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
    p.x = min_x;
    p.y = coefficients[0]*min_x + coefficients[1];
    line.points.push_back(p);
    p.x = max_x;
    p.y = coefficients[0]*max_x + coefficients[1];
    line.points.push_back(p);

    publish_rviz_marker_.publish(line);
}

float PathFollower::getPathDirectionAngle()
{
    vectorVector2f points_left, points_right;
    findPathEdgePoints(&points_left, &points_right);

    if (points_left.size() == 0 || points_right.size() == 0) {
        throw Exception("Missing edge points");
    }

    // fit a line to the edge points
    Eigen::Vector2f left_coeff, right_coeff;

    left_coeff  = fitLinear(points_left);
    right_coeff = fitLinear(points_right);


    /////////////////////////////// test marker
//    visualization_msgs::Marker points;
//    points.header.frame_id = "/map";
//    points.ns = "follow_path/path_edge";
//    points.action = visualization_msgs::Marker::ADD;
//    points.pose.orientation.w = 1.0;
//    points.id = 1;
//    points.type = visualization_msgs::Marker::POINTS;
//    // POINTS markers use x and y scale for width/height respectively
//    points.scale.x = 0.05;
//    points.scale.y = 0.05;
//    // Points are blue
//    points.color.b = 1.0;
//    points.color.a = 1.0;
//    for (vectorVector2f::iterator it = points_left.begin(); it != points_left.end(); ++it) {
//        geometry_msgs::Point p;
//        p.x = (*it)[0];
//        p.y = (*it)[1];
//        p.z = 0;
//        points.points.push_back(p);
//    }
//    publish_rviz_marker_.publish(points);

    std_msgs::ColorRGBA green, blue;
    green.g = 1.0; green.a = 1.0;
    blue.b = 1.0; blue.a = 1.0;
    publishLineMarker(left_coeff, points_left.back()[0]-3, points_left.back()[0]+3, 1, green);
    publishLineMarker(right_coeff, points_right.back()[0]-3, points_right.back()[0]+3, 2, green);

    // middle line m(x) = r(x) + (l(x)-r(x))/2,  (l: left edge line, r: right edge line)
    Eigen::Vector2f mid_coeff( (right_coeff[0] + left_coeff[0])/2, (right_coeff[1] + left_coeff[1])/2 );
    publishLineMarker(mid_coeff, points_right.back()[0]-3, points_right.back()[0]+3, 3, blue);

    return 0.0;
}

bool PathFollower::findPathEdgePoints(vectorVector2f *out_points_left, vectorVector2f *out_points_right)
{
    // Work in frame /map

    /* *** get robot position and direction and orthonormal vector of the direction *** */
    Eigen::Vector2f robot_pos, robot_direction, orthogonal;

    try {
        // position/orientation of the robot
        tf::StampedTransform robot_pose;
        tf_listener_.lookupTransform("/map", "/base_link", ros::Time(0), robot_pose);
        // position
        robot_pos[0] = robot_pose.getOrigin().getX();
        robot_pos[1] = robot_pose.getOrigin().getY();
        // orientation
        btVector3 tmp(1,0,0);
        tmp = tmp.rotate(robot_pose.getRotation().getAxis(), robot_pose.getRotation().getAngle());
        robot_direction[0] = tmp.getX();
        robot_direction[1] = tmp.getY();
    }
    catch (tf::TransformException e) {
        ROS_WARN("tf::TransformException in %s (line %d):\n%s", __FILE__, __LINE__, e.what());
        return false;
    }

    // Orthogonal vector of robot_direction: (x,y) -> (-y,x)
    // Points left of the direction (should be the y-axis)
    orthogonal[0] = - robot_direction[1];
    orthogonal[1] = robot_direction[0];


    /* *** get edge points *** */
    //! Size of the steps when looking for the edge points.
    /** Using map resolution gives the greates possible step size which ensures that we miss no cell. */
    float step_size = map_->info.resolution;

    // go forward
    //! Position of the current forward step.
    Eigen::Vector2f forward_pos = robot_pos - robot_direction;
    for (float forward = -1.0; forward < 1.0; forward += 3*step_size) {
        /** \todo better exception handling here? */
        forward_pos += robot_direction * 3*step_size;

        // break, if obstacle is in front.
        try {
            if (map_->data[transformToMap(forward_pos)] != 0) {
                continue;
            }
        } catch (TransformMapException e) {
            ROS_WARN("Ahead: %s", e.what());
            continue;
        }

        // find left edge
        try {
            Eigen::Vector2f left_edge = forward_pos;
            do {
                left_edge += orthogonal * step_size;
            } while( map_->data[transformToMap(left_edge)] == 0 );
            out_points_left->push_back(left_edge);
        } catch (TransformMapException e) {
            ROS_WARN("Left: %s", e.what());
        }

        // find right edge
        try {
            Eigen::Vector2f right_edge = forward_pos;
            do {
                right_edge += -orthogonal * step_size;
            } while( map_->data[transformToMap(right_edge)] == 0 );
            out_points_right->push_back(right_edge);
        } catch (TransformMapException e) {
            ROS_WARN("Right: %s", e.what());
        }
    }

    // drop last point for it might disturb the line
    if (out_points_left->size())
        out_points_left->pop_back();
    if (out_points_right->size())
        out_points_right->pop_back();



//    if (out_points_right->size()) {
//        PointXYZRGBT a,b;
//        a.x = (out_points_right->front())[0];
//        a.y = (out_points_right->front())[1];
//        b.x = (out_points_right->back())[0];
//        b.y = (out_points_right->back())[1];
//        a.z = b.z = 0;
//        std_msgs::Header h;
//        h.frame_id = "/map";
//        publishTraversaleLineMarker(a,b,h);
//    }


    return true;
}

size_t PathFollower::transformToMap(Eigen::Vector2f point)
{
    int x, y;
    x = (point[0] - map_->info.origin.position.x) / map_->info.resolution;
    y = (point[1] - map_->info.origin.position.y) / map_->info.resolution;

    if (x < 0 || x >= (int)map_->info.width || y < 0 || y >= (int)map_->info.height) {
        throw TransformMapException();
    }

    return y * map_->info.width + x;
}

Eigen::Vector2f PathFollower::fitLinear(const PathFollower::vectorVector2f &points)
{
    using namespace Eigen;

    MatrixXf A(points.size(), 2);
    VectorXf b(points.size());
    Vector2f x;

    // write points to A and b, so that y1 = a*x1+b, y2 = ax2+b, ... is equivalent to Ax = b.
    for (size_t i = 0; i < points.size(); ++i) {
        A(i,0) = points[i][0];
        A(i,1) = 1;
        b(i)   = points[i][1];
    }

    // solve Ax = b using least squares.
    JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);
    x = svd.solve(b);

    //ROS_DEBUG("fitLinear: Coefficents: a = %f, b = %f", x[0], x[1]);

    return x;
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
