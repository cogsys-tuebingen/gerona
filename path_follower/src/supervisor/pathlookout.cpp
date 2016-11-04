#include <path_follower/supervisor/pathlookout.h>

#define DEBUG_PATHLOOKOUT 0

#if DEBUG_PATHLOOKOUT
    #include <opencv2/highgui/highgui.hpp>
#endif
#include <vector>
#include <string>

#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/transforms.h>

#include <path_follower/utils/obstacle_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <path_follower/utils/pose_tracker.h>


using namespace std;

namespace {
//! Module name, that is used for ros console output
const std::string MODULE = "s_pathlookout";
}

PathLookout::PathLookout(PoseTracker &pose_tracker):
    obstacle_frame_("map"),
    pose_tracker_(pose_tracker)
{
    #if DEBUG_PATHLOOKOUT
        cv::namedWindow("Map", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("Path", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("Intersection", CV_WINDOW_KEEPRATIO);
    #endif

    visualizer_ = Visualizer::getInstance();
}

void PathLookout::setObstacleCloud(const std::shared_ptr<ObstacleCloud const> &cloud)
{
    obstacle_cloud_ = cloud;
}

void PathLookout::setPath(Path::ConstPtr path)
{
    // only use path from the last waypoint on ("do not look behind")
    path_.wps.clear();
    if (path->getWaypointIndex() == 0) {
        path_ = path->getCurrentSubPath();
    } else {
        std::vector<Waypoint>::const_iterator start = path->getCurrentSubPath().begin();
        start += (path->getWaypointIndex()-1);
        path_.wps.assign(start, (std::vector<Waypoint>::const_iterator) path->getCurrentSubPath().end());
    }
}

void PathLookout::supervise(State &state, Supervisor::Result *out)
{
    setPath(state.path);
    setObstacleCloud(state.obstacle_cloud);

    // hope for the best
    out->can_continue = true;

    if (!obstacle_cloud_) {
        ROS_WARN_THROTTLE_NAMED(1, MODULE, "PathLookout has not received a valid obstacle cloud. No obstacle lookout is done.");
        return;
    }
    if (path_.empty()) {
        ROS_WARN_THROTTLE_NAMED(1, MODULE, "PathLookout has not received any path yet. No obstacle lookout is done.");
        return;
    }

    vector<Obstacle> observed_obstacles = lookForObstacles();

    // Update tracker
    tracker_.update(observed_obstacles);

    vector<ObstacleTracker::TrackedObstacle> tracked_obs = tracker_.getTrackedObstacles();
    if (tracked_obs.empty()) {
        // no obstacles --> everything is fine.
        return;
    }


    /* The following loop does three things at once:
     *  - calculate weight for each obstacle
     *  - determine max. weight
     *  - convert to obstacle-messages for feedback and visualization
     */

    // Calculaten is done with cv::Points, so robot pose has to be converted.
    cv::Point2f robot_pos(state.robot_pose[0], state.robot_pose[1]);

    vector<path_msgs::Obstacle> obstacle_msgs;
    obstacle_msgs.reserve(tracked_obs.size());

    float max_weight = -1000.0f;
    vector<ObstacleTracker::TrackedObstacle>::const_iterator it;
    for (it = tracked_obs.begin(); it != tracked_obs.end(); ++it) {
        // weight
        float w = weightObstacle(robot_pos, *it);
        max_weight = max(max_weight, w);

        // convert to message
        path_msgs::Obstacle o_msg = it->obstacle().toMsg();
        o_msg.weight = w;
        obstacle_msgs.push_back(o_msg);
    }


    // report obstacles via feedback
    state.feedback.obstacles_on_path = obstacle_msgs;


    // visualize obstacles in rviz
    if (visualizer_->hasSubscriber()) {
        for(size_t i = 0; i < tracked_obs.size(); ++i) {
            geometry_msgs::Point gp = obstacle_msgs[i].position;
            visualizer_->drawCircle(tracked_obs[i].id(), gp, tracked_obs[i].obstacle().radius,
                                    obstacle_frame_, "obstacleonpath", 1,0,0,0.5, 0.1);

            // show the weight.
            stringstream s;
            s << obstacle_msgs[i].weight;
            gp.z = 0.5;
            visualizer_->drawText(i, gp, s.str(), "obstacleonpath_weight", 1,0,0, obstacle_frame_, 0.1);
        }
    }

    // report obstacle, if the highest weight is higher than the defined limit.
    const float limit = 1.0f;
    ROS_DEBUG_NAMED(MODULE, "Max Obstacle Weight: %g, limit: %g", max_weight, limit);
    if (max_weight > limit) {
        ROS_WARN_NAMED(MODULE, "Max Obstacle Weight: %g, limit: %g", max_weight, limit);
        out->can_continue = false;
        out->status = path_msgs::FollowPathResult::RESULT_STATUS_OBSTACLE;
    }
}

void PathLookout::eventNewGoal()
{
    reset();
}

vector<Obstacle> PathLookout::lookForObstacles()
{
    list<cv::Point2f> obs_front = findObstaclesInCloud(obstacle_cloud_);

    // cluster
    list<list<cv::Point2f> > obstacle_points = clusterPoints(obs_front,
                                                             opt_.scan_cluster_max_distance());

    vector<Obstacle> observed_obstacles;
    observed_obstacles.reserve(obstacle_points.size());
    for (auto it = obstacle_points.cbegin(); it != obstacle_points.cend(); ++it) {
        // ignore clusters, that are too small
        if (it->size() < (size_t)opt_.min_number_of_points()) {
            continue;
        }

        try {
            // get obstacle position and size from enclosing circle
            Obstacle obstacle;
            vector<cv::Point2f> v_obs_points(it->begin(), it->end());
            cv::minEnclosingCircle(v_obs_points, obstacle.center, obstacle.radius);

            observed_obstacles.push_back(obstacle);
        } catch (const tf::TransformException& ex) {
            ROS_ERROR_NAMED(MODULE, "TF-Error. Could not transform obstacle position. %s", ex.what());
        } catch (const std::runtime_error &ex) { // is thrown, if no obstacle map is available.
            ROS_ERROR_NAMED(MODULE, "An error occured: %s", ex.what());
        }
    }

    return observed_obstacles;
}

void PathLookout::reset()
{
    // important! path has to be reseted, otherwise obstacles will be readded immediately.
    path_.wps.clear();
    tracker_.reset();
}

std::list<std::list<cv::Point2f> > PathLookout::clusterPoints(const std::list<cv::Point2f> &points,
                                                              const float dist_threshold)
{
    if (points.empty())
        return std::list<std::list<cv::Point2f> >();

    auto get_x = [](const cv::Point2f &p) { return p.x; };
    auto get_y = [](const cv::Point2f &p) { return p.y; };
    std::list<std::list<cv::Point2f> > clusters, x_clusters;

    // cluster along x-axis
    x_clusters = clusterPointsAlongAxis(points, dist_threshold, get_x); // O(n log n), n = |points|

    // cluster each cluster along y-axis and merge results together in `clusters`
    for (const auto &cluster: x_clusters) {
        std::list<std::list<cv::Point2f> > y_clusters = clusterPointsAlongAxis(cluster, dist_threshold, get_y); // O(|c| log |c|)
        clusters.splice(clusters.end(), std::move(y_clusters));    // O(1)
    }

    return clusters;
}

float PathLookout::weightObstacle(cv::Point2f robot_pos, ObstacleTracker::TrackedObstacle o) const
{
    float dist_to_robot = cv::norm(robot_pos - o.obstacle().center) - o.obstacle().radius;
    ros::Duration lifetime = ros::Time::now() - o.time_of_first_sight();

    // w_time is increasing quadratically with the time. t = scale => w_t = 1
    float w_time = pow(lifetime.toSec()/opt_.scale_obstacle_lifetime(), 2);
    // w_dist is increasing quadratically with decreasing distance. d = scale => w_d = 1.
    // For d > 2*scale, w_d is zero (-> cuts off the other half of the parable).
    float w_dist = dist_to_robot < 2*opt_.scale_obstacle_distance()
                   ? pow((dist_to_robot - 2*opt_.scale_obstacle_distance() )/opt_.scale_obstacle_distance(), 2)
                   : 0;

    //ROS_DEBUG_NAMED(MODULE, "WEIGHT: d = %g, t = %g, wd = %g, wt = %g", dist_to_robot, lifetime.toSec(), w_dist, w_time);

    return w_dist + w_time;
}

std::list<cv::Point2f> PathLookout::findObstaclesInCloud(const std::shared_ptr<ObstacleCloud const> &obstacles_container)
{    
    if (path_.size() < 2) {
        ROS_WARN_NAMED(MODULE, "Path has less than 2 waypoints. No obstacle lookout is done.");
        return std::list<cv::Point2f>(); // return empty cloud
    }

    ObstacleCloud::Cloud::ConstPtr cloud = obstacles_container->cloud;

    //TODO: ensure that obstacle_frame_ is the frame of the path.
    ObstacleCloud::Cloud trans_cloud;
    bool has_tf = pose_tracker_.getTransformListener().waitForTransform(obstacle_frame_,
                                                                        cloud->header.frame_id,
                                                                        pcl_conversions::fromPCL(cloud->header.stamp),
                                                                        ros::Duration(0.05));
    if (!has_tf) {
        ROS_WARN_THROTTLE_NAMED(0.5, MODULE, "Got no transfom for obstacle cloud. %s to %s ",obstacle_frame_.c_str(),
                        cloud->header.frame_id.c_str());
        return std::list<cv::Point2f>(); // return empty cloud
    }
    if (!pcl_ros::transformPointCloud(obstacle_frame_, *cloud, trans_cloud, pose_tracker_.getTransformListener())) {
        ROS_ERROR_THROTTLE_NAMED(1.0, MODULE, "Failed to transform obstacle cloud");
        return std::list<cv::Point2f>(); // return empty cloud
    }

    //! Contains an 'is obstacle' flag for each point in the cloud
    vector<bool> is_point_obs(trans_cloud.size(), false);

    cv::Point2f a(path_[0].x, path_[0].y);

    // iterate over second to last point, making steps of size opt_.segment_step_size() (i.e.
    // 'step_size' many segments are approximated by one bigger segment).
    // To assure, that every part of the path is checked, make the first step shorter, if the
    // length of the path is not divisable by the step size.
    size_t first_step = (path_.size() - 1) % opt_.segment_step_size();
    if (first_step == 0) {
        first_step = opt_.segment_step_size();
    }
    size_t first_idx = 0;//min(first_step, path_.size()-1);
 //   size_t first_idx = path_->getWaypointIndex();
    float dist = 0.0;
    cv::Point2f prev(path_[first_idx].x,path_[first_idx].y);
    ROS_INFO_THROTTLE(1.0, "first wp %f %f", prev.x,prev.y);
  //  size_t i = path_->getWaypointIndex();

    for (size_t i = min(first_step, path_.size()-1); i < path_.size(); i += opt_.segment_step_size()) {
   // for (size_t i = path_->getWaypointIndex(); i < path_.size(); i += opt_.segment_step_size()) {
        cv::Point2f b(path_[i].x, path_[i].y);
        dist += cv::norm(b-prev);
        prev = b;
        if (dist>opt_.lookout_distance()) {
            ROS_INFO_THROTTLE(1.0,"no obstacles");
            break;
        }
        /**
         * For each point, compute the distance of this point to the current path segment. If the distance is too small,
         * the corresponding field in is_point_obs is set to true (= this point is assumed to be an obstacle).
         *
         * Formulas taken from http://www.f09.fh-koeln.de/imperia/md/content/personen/schuh_werner/lehrveranstaltung/abstand
         * Section 3 "Abstand zwischen einem Punkt und einer Strecke":
         *
         * Let A, B be the start and end point of the path segment. To get the distance of a point P to this segment
         * first compute
         *      $ \lambda = \frac{\overrightarrow{AP} \cdot \overrightarrow{AB}}{\overrightarrow{AB}^2} $
         * If \lambda < 0:          dist = |\overrightarrow{AP}|
         * If \lambda > 0:          dist = |\overrightarrow{BP}|
         * If \lambda \in [0,1]:    dist = |\overrightarrow{FP}|,  where F = A + \lambda \cdot \overrightarrow{AB}
         */

        // precompute AB and AB^2
        const cv::Point2f ab = b - a;
        const float ab2 = ab.dot(ab);

        // check distance of each point
        for (size_t i = 0; i < trans_cloud.size(); ++i) {
            const cv::Point2f p(trans_cloud.at(i).x, trans_cloud.at(i).y);
            const cv::Point2f ap = p - a;
            const float lambda = ap.dot(ab) / ab2;

            float dist;
            if (lambda < 0) {
                // A is nearest point
                dist = cv::norm(ap);
            } else if (lambda > 1) {
                // B is nearest point
                dist = cv::norm(p - b);
            } else {
                // F is nearest point
                cv::Point2f f = a + lambda * ab;
                dist = cv::norm(p - f);
            }

            if (dist < opt_.path_width()/2.0) {
                is_point_obs[i] = true;
            }
        }

        a = b;
    }

    list<cv::Point2f> obstacle_points;
    for (size_t i = 0; i < trans_cloud.size(); ++i) {
        if (is_point_obs[i]) {
            cv::Point2f p(trans_cloud.at(i).x, trans_cloud.at(i).y);
            obstacle_points.push_back(p);
        }
    }

    return obstacle_points;
}

std::list<std::list<cv::Point2f> > PathLookout::clusterPointsAlongAxis(std::list<cv::Point2f> points,
                                                                       const float dist_threshold,
                                                                       std::function<float (const cv::Point2f &)> get_value)
{
    auto dist = [&get_value](const cv::Point2f &a, const cv::Point2f &b) {
        return abs(get_value(a) - get_value(b));
    };

    // sort points along axis   -> O(n log(n))
    points.sort([&get_value](const cv::Point2f &a, const cv::Point2f &b) {
        return get_value(a) < get_value(b);
    });

    list<list<cv::Point2f> > clusters;
    list<cv::Point2f> current_cluster;

    // add first point to the current cluster
    auto p_it = points.cbegin();
    current_cluster.push_back(*p_it);

    // iterate over the remaining points
    for (++p_it; p_it != points.end(); ++p_it) {
        if (dist(current_cluster.back(), *p_it) > dist_threshold) {  // dist and `>` are O(1)
            // start new cluster
            clusters.push_back(std::move(current_cluster));          // O(1)
            current_cluster.clear(); // clear has linear complexity, but after moving the
                                     // content, the list is empty --> O(1)
        }

        current_cluster.push_back(*p_it);                            // O(1)
    } // loop: O(n)
    clusters.push_back(std::move(current_cluster));                  // O(1)

    return clusters;
}
