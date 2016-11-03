
#include <path_follower/obstacle_avoidance/obstacledetectorpatsy.h>

#include <Eigen/Core>
//#include <opencv2/opencv.hpp> // only for debugging
#include <path_follower/utils/visualizer.h>
#include <path_follower/controller/robotcontrollertrailer.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <path_follower/utils/visualizer.h>

using namespace Eigen;


namespace {
//! Module name, that is used for ros console output
const std::string MODULE = "obstacle_avoider";
}

ObstacleDetectorPatsy::ObstacleDetectorPatsy(const tf::TransformListener *tf_listener, RobotControllerTrailer *ctrl)
    : tf_listener_(tf_listener), robot_controller_(ctrl)
{
    front_frame_.assign("base_link");
    rear_frame_.assign("s300_rear");
}


void ObstacleDetectorPatsy::getPolygon(float width, float length, float course_angle, float curve_enlarge_factor,std::vector<cv::Point2f>& polygon) const
{
    /// Based on http://stackoverflow.com/questions/1217585/parallelogram-contains-point

    /*
     * The parallelogram is defined by four points p,q,r,s:
     * p and q are the corners near the robot, r and s are on the opposite site.
     *
     * For course_angle = 0, the parallelogram is a rectangle. Ifcourse_angle != 0, a and b
     * stay constant, but c is moved to the side, depending on course_angle:
     *
     * course_angle = 0:
     *          p------------------r
     *  ##   ## |                  | |
     *  ####### |                  | width
     *  ####### |                  | |
     *  ##   ## |                  |
     *          q------------------s
     *   ^robot    <-  length  ->
     *
     *
     * course_angle < 0:
     *          p---+
     *  ##   ## |    +--+
     *  ####### |        +--+
     *  ####### |            +--r
     *  ##   ## |               |
     *          |               |
     *          |               |
     *          q---+           |
     *               +--+       |
     *                   +--+   |
     *                       +--s
     *
     * The Box is enlarged toward the inside curve. In the above example, q is moved away from
     * the robot, while p is fixed. For course_angle > 0 it is vice verca.
     * The amount of this enlarging is controlled by the argument 'curve_enlarge_factor'.
     */


    Vector2f p(0.0f,  width/2.0f);
    Vector2f q(0.0f, -width/2.0f);

    course_angle *= 0.5f;

    float sin_angle = std::sin(course_angle);
    float cos_angle = std::cos(course_angle);

    // stretch box sidewards, depending on course angle
    if (course_angle > 0) {
        p(1) += curve_enlarge_factor * sin_angle;
    } else if (course_angle < 0) {
        q(1) += curve_enlarge_factor * sin_angle;
    }

    Vector2f r = p + length * Vector2f(cos_angle, sin_angle);
    Vector2f s = q + length * Vector2f(cos_angle, sin_angle);

    polygon.push_back( cv::Point2f(p[0], p[1]) );
    polygon.push_back( cv::Point2f(q[0], q[1]) );
    polygon.push_back( cv::Point2f(s[0], s[1]) );
    polygon.push_back( cv::Point2f(r[0], r[1]) );

}



bool ObstacleDetectorPatsy::avoid(MoveCommand * const cmd,
                             ObstacleCloud::ConstPtr obstacles,
                             const ObstacleAvoider::State &state)
{
    float course = cmd->getDirectionAngle(); //TODO: use CoursePredictor instead of command?


    //! Factor which defines, how much the box is enlarged in curves.
    const float enlarge_factor = 0.5; // should this be a parameter?

    /* Calculate length of the collision box, depending on current velocity.
     * v <= v_min:
     *   length = min_length
     * v > v_min && v < v_sat:
     *   length  interpolated between min_length and max_length:
     *   length = min_length + FACTOR * (max_length - min_length) * (v - v_min) / (v_sat - v_min)
     * v >= v_sat:
     *   length = max_length
     */
    float v = cmd->getVelocity();

    const float diff_to_min_velocity = v - state.parameters.min_velocity();

    float vel_saturation = opt_.velocity_saturation() > 0
            ? opt_.velocity_saturation()
            : state.parameters.max_velocity();
    const float norm = vel_saturation - state.parameters.min_velocity();
    const float span = opt_.max_length() - opt_.min_length();
    const float interp = std::max(0.0f, diff_to_min_velocity) / std::max(norm, 0.001f);
    const float f = std::min(1.0f, opt_.velocity_factor() * interp);

    float box_length = opt_.min_length() + span * f;

    //ROS_DEBUG_NAMED(MODULE, "Collision Box: v = %g -> len = %g", v, box_length);

    double distance_to_goal = state.path->getCurrentSubPath().back().distanceTo(state.path->getCurrentWaypoint());

    if(box_length > distance_to_goal) {
        box_length = distance_to_goal + 0.2;
    }

    if(box_length < opt_.crit_length()) {
        box_length = opt_.crit_length();
    }

    if (v<0) {
        box_length*=-1.0;
    }

    bool collision = checkOnCloud(obstacles, opt_.width(),
                                  box_length, course, enlarge_factor);


    if(collision) {
        // stop motion
        cmd->setVelocity(0);
    }
    return collision;
}




bool ObstacleDetectorPatsy::checkOnCloud(ObstacleCloud::ConstPtr obstacles, float width, float length,  float course_angle, float curve_enlarge_factor)
{
    bool collision = false;
    bool backwards = false;
    if (length<0) {
        backwards = true;
        length*=-1.0;
    }
    std::vector<cv::Point2f> polygon;
    getPolygon(width, length, course_angle, curve_enlarge_factor, polygon);

    if (polygon.size() == 0) {
        ROS_WARN_NAMED(MODULE, "Obstacle polygon is empty -> no obstacle test is done!");
        return false;
    }
    geometry_msgs::PointStamped gm_p, gm_pt;
    // transform the polygon to the obstacle cloud frame
    if (backwards) {
        gm_p.header.frame_id = rear_frame_;
    } else {
        gm_p.header.frame_id = front_frame_;
    }
    gm_p.header.stamp = pcl_conversions::fromPCL(obstacles->header.stamp);



    try {


        for (cv::Point2f &p : polygon) {
            gm_p.point.x = p.x;
            gm_p.point.y = p.y;
            tf_listener_->transformPoint(obstacles->header.frame_id, gm_p, gm_pt);
            p.x = gm_pt.point.x;
            p.y = gm_pt.point.y;
        }
    } catch (tf::TransformException& ex) {
        ROS_ERROR_NAMED(MODULE, "Failed to transform polygon to obstacle cloud frame: %s", ex.what());
        // can't check for obstacles, so better assume there is one.
        return true;
    }


    /// now check each point of the scan
    ObstacleCloud::const_iterator point_it;
    for (point_it = obstacles->begin(); point_it != obstacles->end(); ++point_it) {
        // check if this scan point is inside the polygon
        cv::Point2f point( point_it->x, point_it->y );

        if (cv::pointPolygonTest(polygon, point, false) == 1) {
            collision = true;
            break; // no need to check the remaining points
        }
    }


    // visualization
    visualize(polygon,obstacles->header.frame_id, collision);


    return collision;
}



void ObstacleDetectorPatsy::visualize(std::vector<cv::Point2f>& polygon,const std::string& frame,
                                        bool hasObstacle) const
{
    Visualizer* vis = Visualizer::getInstance();
    if (vis->hasSubscriber()) {
        // push first element to back, to close the polygon.
        polygon.push_back(polygon.front());
        std::vector<cv::Point2f>::iterator it = polygon.begin();
        cv::Point2f p1 = *it;

        int marker_id = 0;
        for (++it; it != polygon.end(); ++it) {
            cv::Point2f p2 = *it;

            // convert cv::Point2f to ros points...
            geometry_msgs::Point gp1, gp2;
            gp1.x = p1.x;  gp1.y = p1.y;
            gp2.x = p2.x;  gp2.y = p2.y;

            // colour is green when the box is empty and red if there is an obstacle
            float r = hasObstacle ? 1 : 0;
            float g = 1 - r;
            vis->drawLine(marker_id++, gp1, gp2, frame, "collision_box", r,g,0, 0.1, 0.05);

            p1 = p2;
        }
    }
}
