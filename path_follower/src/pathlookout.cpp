#include "pathlookout.h"

#define DEBUG_PATHLOOKOUT 0

#if DEBUG_PATHLOOKOUT
    #include <opencv2/highgui/highgui.hpp>
#endif
#include <vector>
#include <string>
#include "pathfollower.h"
#include "boost/foreach.hpp"

#include <laser_geometry/laser_geometry.h>
#include <opencv2/flann/flann.hpp>

using namespace std;

PathLookout::PathLookout(PathFollower *node):
    node_(node),
    obstacle_frame_("/map")
{
    #if DEBUG_PATHLOOKOUT
        cv::namedWindow("Map", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("Path", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("Intersection", CV_WINDOW_KEEPRATIO);
    #endif

    visualizer_ = Visualizer::getInstance();
    configure();
}

void PathLookout::setScan(const sensor_msgs::LaserScanConstPtr &msg, bool isBack)
{
    if (isBack) {
        back_scan_ = msg;
    } else {
        front_scan_ = msg;
    }
}

void PathLookout::setMap(const nav_msgs::OccupancyGridConstPtr &map)
{
    map_ = map;
    map_trans_.setMap(map);

    // convert to image
    map_image_ = cv::Mat(map->info.height, map->info.width, CV_8UC1);
    // Directly copy. Now 0 -> free, >0 -> obstacle. For visualization, it would usually be inverted, here, however,
    // this is perfectly fine, as the obstacles have to be "true" for the intersection with the path image.
    // Note, that it is important, that the path image is then really binary with only 0 and 255, to make sure, that the
    // bitwise_and can not miss any obstacle pixels.
    std::copy(map->data.begin(), map->data.end(), map_image_.data);
    //map_image_.data = &map->data[0];  <- no copy, only references. Would be nice, but yields problems with const...
}

void PathLookout::setPath(const PathWithPosition &path)
{
    // only use path from the last waypoint on ("do not look behind")
    Path path_ahead;
    if (path.wp_idx == 0) {
        path_ahead = *path.current_path;
    } else {
        Path::const_iterator start = path.current_path->begin();
        start += (path.wp_idx-1);
        path_ahead.assign(start, (Path::const_iterator) path.current_path->end());
    }

    path_ = path_ahead;

    // if there is no map yet, we do not know the size of the path image (and do not need it anyway)
    if (map_) {
        drawPathToImage(path_ahead);
    }
}

bool PathLookout::lookForObstacles(path_msgs::FollowPathFeedback *feedback)
{
    if (!map_ && !front_scan_ && !back_scan_) {
        ROS_WARN_THROTTLE(1, "PathLookout has not received any map or scan yet. No obstacle lookout is done.");
        return false;
    }
    if (path_.empty()) {
        ROS_WARN_THROTTLE(1, "PathLookout has not received any path yet. No obstacle lookout is done.");
        return false;
    }


    vector<Obstacle> observed_obstacles;
    if (map_) {
        observed_obstacles = lookForObstaclesInMap();
    }

    if (front_scan_ || back_scan_) {
        vector<Obstacle> obs = lookForObstaclesInScans();
        observed_obstacles.insert(observed_obstacles.end(), obs.begin(), obs.end());
    }


    // Update tracker
    tracker_.update(observed_obstacles);

    vector<ObstacleTracker::TrackedObstacle> tracked_obs = tracker_.getTrackedObstacles();
    if (tracked_obs.empty()) {
        return false;
    }


    /* The following loop does three things at once:
     *  - calculate weight for each obstacle
     *  - determine max. weight
     *  - convert to obstacle-messages for feedback and visualization
     */

    // Calculaten is done with cv::Points, so robot pose has to be converted.
    Eigen::Vector3d robot_pose_eigen = node_->getRobotPose();
    cv::Point2f robot_pos(robot_pose_eigen[0], robot_pose_eigen[1]);

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
    if (feedback != NULL) {
        feedback->obstacles_on_path = obstacle_msgs;
    }


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
    ROS_DEBUG("Max Obstacle Weight: %g, limit: %g", max_weight, opt_.obstacle_weight_limit_);
    return max_weight > opt_.obstacle_weight_limit_;
}

vector<Obstacle> PathLookout::lookForObstaclesInMap()
{
    // Calculate intersection of the obstacle map and the path ==> provides the obstacles on the path
    cv::Mat intersect;
    cv::bitwise_and(map_image_, path_image_, intersect);

    // find obstacle contours on the path
    vector<vector<cv::Point> > contours;
    cv::findContours(intersect, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Get center of each obstacle (for tracking)
    vector<Obstacle> observed_obstacles;
    observed_obstacles.reserve(contours.size());
    for(size_t i = 0; i < contours.size(); ++i) {
        try {
            // get obstacle position and size from enclosing circle
            Obstacle obstacle;
            cv::minEnclosingCircle(contours[i], obstacle.center, obstacle.radius);

            #if DEBUG_PATHLOOKOUT
                // dont need intersect anymore, so it is ok to draw debug stuff to it.
                cv::circle(intersect, obstacle.center, obstacle.radius, cv::Scalar(255));
            #endif

            // transform center and scale radius from pixel to meters.
            obstacle.center = map_trans_.transformPointFromMap(obstacle.center, obstacle_frame_);
            obstacle.radius *= map_->info.resolution;

            observed_obstacles.push_back(obstacle);
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("TF-Error. Could not transform obstacle position. %s", ex.what());
        } catch (const std::runtime_error &ex) { // is thrown, if no obstacle map is available.
            ROS_ERROR("An error occured: %s", ex.what());
        }
    }

    // debug
    #if DEBUG_PATHLOOKOUT
        cv::imshow("Map", map_image_);
        cv::imshow("Path", path_image_);
        cv::imshow("Intersection", intersect);
        cv::waitKey(5);
    #endif

    return observed_obstacles;
}

vector<Obstacle> PathLookout::lookForObstaclesInScans()
{
    vector<cv::Point2f> obs_front, obs_back;
    if (front_scan_) {
        obs_front = findObstaclesInScan(front_scan_);
    }
    if (back_scan_) {
        obs_back = findObstaclesInScan(back_scan_);
    }

    // merge result of front and back scan
    obs_front.insert(obs_front.end(), obs_back.begin(), obs_back.end());

    // cluster
    vector<vector<cv::Point2f> > obstacle_points = clusterPoints(obs_front);

    vector<Obstacle> observed_obstacles;
    observed_obstacles.reserve(obstacle_points.size());
    for (vector<vector<cv::Point2f> >::const_iterator it = obstacle_points.begin(); it != obstacle_points.end(); ++it) {
        // ignore clusters, that are too small
        if (it->size() < opt_.min_number_of_points_) {
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
     * the threshold defined by opt_.scan_cluster_max_distance_. In this case, the current cluster is closed and a
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

        if (d > opt_.scan_cluster_max_distance_) {
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
    path_image_ = cv::Scalar(0);
    tracker_.reset();
}

void PathLookout::configure()
{
    ros::param::param<float>("~obstacle_scale_distance", opt_.scale_obstacle_distance_, 1.0f);
    ros::param::param<float>("~obstacle_scale_lifetime", opt_.scale_obstacle_lifetime_, 10.0f);
    ros::param::param<float>("~path_width", opt_.path_width_, 0.5f);
    ros::param::param<int>("~segment_step_size", opt_.segment_step_size_, 5);
    ros::param::param<float>("~scan_cluster_max_distance", opt_.scan_cluster_max_distance_, 0.5f);
    ros::param::param<int>("~min_number_of_points", opt_.min_number_of_points_, 3);

    // there should be no need to make max. weight configurable, as it can be scaled using the parameters above.
    opt_.obstacle_weight_limit_ = 1.0f;
}


void PathLookout::drawPathToImage(const Path &path)
{
    /// This method assumes, that the map is already set!

    // initialize path image to fit map size
    if (path_image_.empty()) {
        path_image_ = cv::Mat(map_->info.height, map_->info.width, CV_8UC1, cv::Scalar(0));
    }

    // first reset image
    path_image_ = cv::Scalar(0); //<- yup, this is possible in OpenCV :) Sets every pixel to 0.

    if (path.size() < 2) {
        ROS_WARN("Path has less than 2 waypoints. No obstacle lookout is done.");
        return;
    }

    int path_width_pixel = (int) ceil(opt_.path_width_ / map_->info.resolution);

    Path::const_iterator iter = path.begin();
    try {
        // get first point
        cv::Point2f p1(iter->x, iter->y);
        p1 = map_trans_.transformPointToMap(p1, "/map");
        // iterate over second to last point
        for (++iter; iter != path.end(); ++iter) {
            cv::Point2f p2(iter->x, iter->y);
            p2 = map_trans_.transformPointToMap(p2, "/map");

            cv::line(path_image_, p1, p2, cv::Scalar(255), path_width_pixel);
            p1 = p2;
        }
    } catch (const tf::TransformException& ex) {
        ROS_ERROR("Failed to transform path to map in PathLookout. TF-Exception: %s\n PathLookout will not be able to check for obstacles on the path!", ex.what());
        // Nothing to do here. Maybe parts of the path have been drawed to the map before the exception occured.
        // In this case the lookout will work as intended but only check the first part of the path.
        // In the worst case, nothing was drawn, then the lookout will simply not see obstacles, but it will not crash
        // in any way.
    } catch (const std::runtime_error &ex) { // is thrown, if no obstacle map is available.
        ROS_ERROR("An error occured: %s", ex.what());
    }
}

float PathLookout::weightObstacle(cv::Point2f robot_pos, ObstacleTracker::TrackedObstacle o) const
{
    float dist_to_robot = cv::norm(robot_pos - o.obstacle().center) - o.obstacle().radius;
    ros::Duration lifetime = ros::Time::now() - o.time_of_first_sight();

    // w_time is increasing quadratically with the time. t = scale => w_t = 1
    float w_time = pow(lifetime.toSec()/opt_.scale_obstacle_lifetime_, 2);
    // w_dist is increasing quadratically with decreasing distance. d = scale => w_d = 1.
    // For d > 2*scale, w_d is zero (-> cuts off the other half of the parable).
    float w_dist = dist_to_robot < 2*opt_.scale_obstacle_distance_
                   ? pow((dist_to_robot - 2*opt_.scale_obstacle_distance_ )/opt_.scale_obstacle_distance_, 2)
                   : 0;

    //ROS_DEBUG("WEIGHT: d = %g, t = %g, wd = %g, wt = %g", dist_to_robot, lifetime.toSec(), w_dist, w_time);

    return w_dist + w_time;
}

std::vector<cv::Point2f> PathLookout::findObstaclesInScan(const sensor_msgs::LaserScanConstPtr &scan)
{
    if (path_.size() < 2) {
        ROS_WARN("Path has less than 2 waypoints. No obstacle lookout is done.");
        return std::vector<cv::Point2f>(); // return empty cloud
    }

    laser_geometry::LaserProjection proj;
    sensor_msgs::PointCloud cloud;

    try {
        if(!tf_listener_.waitForTransform(
                    scan->header.frame_id,
                    "/map",
                    scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                    ros::Duration(1.0))) {
            ROS_ERROR("[PathLookout] No transform for laser scan from %s to /map. No obstacle lookout is done!", scan->header.frame_id.c_str());
            return std::vector<cv::Point2f>(); // return empty cloud
        }

        proj.transformLaserScanToPointCloud("/map", *scan, cloud, tf_listener_, scan->range_max-0.5, laser_geometry::channel_option::Index);
    } catch (const tf::TransformException& ex) {
        ROS_ERROR("[PathLookout] Failed to transform scan. TF-Exception: %s\n PathLookout will not be able to check for obstacles on the path!", ex.what());
        return std::vector<cv::Point2f>(); // return empty cloud
    }


    //! Contains an 'is obstacle' flag for each point in the cloud
    vector<bool> is_point_obs(cloud.points.size(), false);

    cv::Point2f a(path_[0].x, path_[0].y);

    // iterate over second to last point, making steps of size opt_.segment_step_size_ (i.e. 'step_size' many segments
    // are approximated by one bigger segment).
    //FIXME: the last waypoints are ignored, if step size does not fit.
    for (size_t i = opt_.segment_step_size_; i < path_.size(); i += opt_.segment_step_size_) {
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
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            const cv::Point2f p(cloud.points[i].x, cloud.points[i].y);
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

            if (dist < opt_.path_width_/2.0) {
                is_point_obs[i] = true;
            }
        }

        a = b;
    }

    vector<cv::Point2f> obstacle_points;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        if (is_point_obs[i]) {
            cv::Point2f p(cloud.points[i].x, cloud.points[i].y);
            obstacle_points.push_back(p);
        }
    }

    return obstacle_points;
}
