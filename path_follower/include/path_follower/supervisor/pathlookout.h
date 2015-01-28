#ifndef PATHLOOKOUT_H
#define PATHLOOKOUT_H

/// STL
#include <string>

/// THIRD PARTY
#include <opencv2/imgproc/imgproc.hpp>

/// ROS
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

/// PROJECT
#include <path_follower/supervisor/supervisor.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/obstaclecloud.hpp>
#include <path_follower/utils/visualizer.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/supervisor/obstacletracker.h>
#include <path_msgs/FollowPathFeedback.h>


/**
 * @brief Looks out for obstacles on the path.
 *
 * Looks for obstacles farther ahead, that are blocking the path. Keeps track of such obstacles and weights them,
 * depending on how long the obstacle is visible and how far away it is.
 * If the weight of any obstacle exceeds a defined limit, an alarm is risen so the robot can early react e.g. by
 * replaning.
 *
 * The weight is calculated in weightObstacle(). It increases quadratically with increasing lifetime and decreasing
 * distance.
 * There are two parameters to adjust the weight:
 *    ~obstacle_scale_distance: Distance at which the robot stops, independed of the duration-weight.
 *    ~obstacle_scale_lifetime: Duration after which the robot stops, independend of the distance-weight.
 */
class PathLookout : public Supervisor
{
public:
    PathLookout(const tf::TransformListener *tf_listener);

    virtual std::string getName() const {
        return "PathLookout";
    }

    void setObstacleCloud(const ObstacleCloud::ConstPtr &cloud);

    //! Check if there is an obstacle on the path ahead of the robot, that gives a reason to cancel the current path.
    virtual void supervise(State &state, Result *out);

    virtual inline void eventNewGoal();

    //! Reset the obstacle tracker (should be called before starting a new path).
    void reset();

private:
    struct Options : public Parameters
    {
        P<float> scale_obstacle_distance;
        P<float> scale_obstacle_lifetime;
        P<float> path_width;
        P<int> segment_step_size;
        P<float> scan_cluster_max_distance;
        P<int> min_number_of_points;

        //! Stop robot, if the weight of an obstacle becomes higher than this value.
        //P<float> obstacle_weight_limit;

        Options():
            scale_obstacle_distance(this, "~supervisor/path_lookout/obstacle_scale_distance",  1.0f, ""),
            scale_obstacle_lifetime(this, "~supervisor/path_lookout/obstacle_scale_lifetime",  10.0f, ""),
            path_width(this, "~supervisor/path_lookout/path_width",  0.5f, "Width of the path in meters (should be at least the width of the robot)."),
            segment_step_size(this, "~supervisor/path_lookout/segment_step_size",  5, "Number of segments that are merged together for speed up."),
            scan_cluster_max_distance(this, "~supervisor/path_lookout/scan_cluster_max_distance",  0.5f, "Maximum distance of a scan point to it's neighbour (in terms of scan angle), to combine them to the same cluster."),
            min_number_of_points(this, "~supervisor/path_lookout/min_number_of_points",  3, "Minimum number of points on one obstacle (smaller clusters are ignored).")
        {}
    } opt_;


    //! TF-Frame in which the obstacles are tracked (should be independent of the robots movement, thus /map is a good choise).
    std::string obstacle_frame_;

    ObstacleTracker tracker_;
    Visualizer *visualizer_;
    const tf::TransformListener *tf_listener_;

    ObstacleCloud::ConstPtr obstacle_cloud_;

    SubPath path_;


    //! Set the path, which is to be checked for obstacles.
    void setPath(Path::ConstPtr path);

    std::vector<Obstacle> lookForObstacles();

    std::vector<std::vector<cv::Point2f> > clusterPoints(const std::vector<cv::Point2f> &points);

    //! Compute weight for the given obstacle, depending on its distance to the robot and its lifetime.
    float weightObstacle(cv::Point2f robot_pos, ObstacleTracker::TrackedObstacle o) const;

    std::vector<cv::Point2f> findObstaclesInCloud(const ObstacleCloud::ConstPtr &cloud);
};

#endif // PATHLOOKOUT_H
