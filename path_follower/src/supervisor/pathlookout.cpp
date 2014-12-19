#include <path_follower/supervisor/pathlookout.h>

#define DEBUG_PATHLOOKOUT 0

#if DEBUG_PATHLOOKOUT
    #include <opencv2/highgui/highgui.hpp>
#endif
#include <vector>
#include <string>
#include <path_follower/pathfollower.h>
#include <boost/foreach.hpp>

#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/transforms.h>
#include <opencv2/flann/flann.hpp>

using namespace std;

PathLookout::PathLookout():
    obstacle_frame_("/map")
{
    #if DEBUG_PATHLOOKOUT
        cv::namedWindow("Map", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("Path", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("Intersection", CV_WINDOW_KEEPRATIO);
    #endif

    visualizer_ = Visualizer::getInstance();

    // FIXME: it is ugly, having to subscribe the same topic at different places...
    ros::NodeHandle node_handle_;
    obstacle_cloud_sub_ = node_handle_.subscribe<ObstacleCloud>("/obstacle_cloud", 10, &PathLookout::setObstacleCloud, this);
}

void PathLookout::setObstacleCloud(const ObstacleCloud::ConstPtr &cloud)
{
    obstacle_cloud_ = cloud;
}

void PathLookout::setPath(Path::ConstPtr path)
{
    // only use path from the last waypoint on ("do not look behind")
    SubPath path_ahead;
    if (path->getWaypointIndex() == 0) {
        path_ahead = path->getCurrentSubPath();
    } else {
        SubPath::const_iterator start = path->getCurrentSubPath().begin();
        start += (path->getWaypointIndex()-1);
        path_ahead.assign(start, (SubPath::const_iterator) path->getCurrentSubPath().end());
    }

    path_ = path_ahead; //TODO: is it possible to do this without copy?
}

void PathLookout::supervise(State &state, Supervisor::Result *out)
{
    setPath(state.path);

    // hope for the best
    out->can_continue = true;

    if (!obstacle_cloud_) {
        ROS_WARN_THROTTLE(1, "PathLookout has not received any obstacle cloud yet. No obstacle lookout is done.");
        return;
    }
    if (path_.empty()) {
        ROS_WARN_THROTTLE(1, "PathLookout has not received any path yet. No obstacle lookout is done.");
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
            // this should be a unique identifier for a tracked obstacle
            //FIXME: scheinbar nicht unique genug...
            //int id = tracked_obs[i].time_of_first_sight().toNSec();

            geometry_msgs::Point gp = obstacle_msgs[i].position;
            //visualizer_->drawMark(id, gp, "obstacleonpath", 1,0,0, obstacle_frame_);
            visualizer_->drawCircle(i, gp, tracked_obs[i].obstacle().radius, obstacle_frame_, "obstacleonpath", 1,0,0,0.5, 0.1);

            // show the weight.
            stringstream s;
            s << obstacle_msgs[i].weight;
            gp.z = 0.5;
            visualizer_->drawText(i, gp, s.str(), "obstacleonpath_weight", 1,0,0, obstacle_frame_, 0.1);
        }
    }

    // report obstacle, if the highest weight is higher than the defined limit.
    const float limit = 1.0f;
    ROS_DEBUG("Max Obstacle Weight: %g, limit: %g", max_weight, limit);
    if (max_weight > limit) {
        out->can_continue = false;
        out->status = path_msgs::FollowPathResult::MOTION_STATUS_OBSTACLE;
    }
}

void PathLookout::eventNewGoal()
{
    reset();
}

vector<Obstacle> PathLookout::lookForObstacles()
{
//    vector<cv::Point2f> obs_front, obs_back;
//    if (front_scan_) {
//        obs_front = findObstaclesInScan(front_scan_);
//    }
//    if (back_scan_) {
//        obs_back = findObstaclesInScan(back_scan_);
//    }

//    // merge result of front and back scan
//    obs_front.insert(obs_front.end(), obs_back.begin(), obs_back.end());

    vector<cv::Point2f> obs_front = findObstaclesInCloud(obstacle_cloud_);

    // cluster
    vector<vector<cv::Point2f> > obstacle_points = clusterPoints(obs_front);

    vector<Obstacle> observed_obstacles;
    observed_obstacles.reserve(obstacle_points.size());
    for (vector<vector<cv::Point2f> >::const_iterator it = obstacle_points.begin(); it != obstacle_points.end(); ++it) {
        // ignore clusters, that are too small
        if (it->size() < (size_t)opt_.min_number_of_points()) {
            continue;
        }

        try {
            // get obstacle position and size from enclosing circle
            Obstacle obstacle;
            cv::minEnclosingCircle(*it, obstacle.center, obstacle.radius);

            observed_obstacles.push_back(obstacle);
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("TF-Error. Could not transform obstacle position. %s", ex.what());
        } catch (const std::runtime_error &ex) { // is thrown, if no obstacle map is available.
            ROS_ERROR("An error occured: %s", ex.what());
        }
    }

    return observed_obstacles;
}

vector<vector<cv::Point2f> > PathLookout::clusterPoints(const vector<cv::Point2f> &points)
{
//    // http://answers.opencv.org/question/8359/hierarchical-clustering-with-flann/
//    cv::Mat in, centers(5, 2);
//    for (vector<cv::Point2f>::iterator it = points.begin; it != points.end; ++it) {
//        in.push_back(*it);
//    }

//    // create the kmeans parameters structure: branching factor = 32,
//    // number of iterations = 100, choose initial centers by PP-algorithm
//    cv::flann::KMeansIndexParams kmean_params(8, 10, cv::flann::FLANN_CENTERS_KMEANSPP);

//    int num_clusters = cv::flann::hierarchicalClustering<cv::L2<float> >(in, centers, kmean_params);

//    centers = centers.rowRange(cv::Range(0,true_number_clusters));


    /* The clustering here is very simple and exploits the fact, that the points of the laser scan are already
     * ordered by their scan angle.
     * Consecutive points are put to the same cluster, until the distance between two neighbouring points exceeds
     * the threshold defined by opt_.scan_cluster_max_distance(). In this case, the current cluster is closed and a
     * new cluster is started.
     *  --> clusters are in fact segments of the scan.
     */

    vector<vector<cv::Point2f> > result;

    if (points.empty()) {
        return result;
    }

    vector<cv::Point2f> cluster;
    cluster.push_back(points[0]);
    for (size_t i = 1; i < points.size(); ++i) {
        float d = cv::norm(points[i-1] - points[i]);

//        // Debug - show point index in rviz
//        geometry_msgs::Point gp;
//        gp.x = points[i].x;
//        gp.y = points[i].y;
//        gp.z = i*0.1;
//        visualizer_->drawText(i, gp, boost::lexical_cast<std::string>(i), "obs_scan_index", 0,1,0);

        if (d > opt_.scan_cluster_max_distance()) {
            // end cluster
            result.push_back(cluster);
            cluster.clear();
        }
        cluster.push_back(points[i]);
    }
    // add last cluster
    result.push_back(cluster);

    ROS_DEBUG("[PathLookout] #points: %zu, #clusters: %zu", points.size(), result.size());

    return result;
}

void PathLookout::reset()
{
    // important! path has to be reseted, otherwise obstacles will be readded immediately.
    path_.clear();
    tracker_.reset();
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

    //ROS_DEBUG("WEIGHT: d = %g, t = %g, wd = %g, wt = %g", dist_to_robot, lifetime.toSec(), w_dist, w_time);

    return w_dist + w_time;
}

std::vector<cv::Point2f> PathLookout::findObstaclesInCloud(const ObstacleCloud::ConstPtr &cloud)
{
    if (path_.size() < 2) {
        ROS_WARN("Path has less than 2 waypoints. No obstacle lookout is done.");
        return std::vector<cv::Point2f>(); // return empty cloud
    }

    //TODO: are cloud and path in the same frame?
    ObstacleCloud trans_cloud;
    try {
        pcl_ros::transformPointCloud(obstacle_frame_, *cloud, trans_cloud, tf_listener_);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Failed to transform obstacle cloud: %s", ex.what());
        return std::vector<cv::Point2f>(); // return empty cloud
    }

    //! Contains an 'is obstacle' flag for each point in the cloud
    vector<bool> is_point_obs(trans_cloud.size(), false);

    cv::Point2f a(path_[0].x, path_[0].y);

    // iterate over second to last point, making steps of size opt_.segment_step_size() (i.e. 'step_size' many segments
    // are approximated by one bigger segment).
    //FIXME: the last waypoints are ignored, if step size does not fit.
    for (size_t i = opt_.segment_step_size(); i < path_.size(); i += opt_.segment_step_size()) {
        cv::Point2f b(path_[i].x, path_[i].y);

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

    vector<cv::Point2f> obstacle_points;
    for (size_t i = 0; i < trans_cloud.size(); ++i) {
        if (is_point_obs[i]) {
            cv::Point2f p(trans_cloud.at(i).x, trans_cloud.at(i).y);
            obstacle_points.push_back(p);
        }
    }

    return obstacle_points;
}
