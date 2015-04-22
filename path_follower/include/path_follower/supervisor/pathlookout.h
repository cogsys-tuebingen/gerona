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

    /**
     * @brief Simple approximate clustering algorithm on 2d points.
     *
     * This is a simple approximate clustering algorithm to assign points of the obstacle cloud
     * to different obstacles (which is required for obstacle tracking).
     *
     * Basic idea
     * ----------
     * The clustering is done axis-wise. First the points are clustered along
     * the x-axis. Then each cluster is again clustered along the y-axis.
     *
     * More detailed:
     * --------------
     *  1) Order points along x-axis
     *  2) Split, when distance between two neighbouring points is greater than `dist_threshold`
     *  3) For each chunk:
     *      4) Order points along y-axis
     *      5) Split like in 2)
     *
     * The result is a set of clusters, where the distance between two clusters is greater than
     * `dist_threshold` along each axis (i.e. `dy > dist_threshold and dx > dist_threshold`).
     * There can be arbitrary many clusters and the clusters can be of arbitrary size.
     *
     * Approximation Error
     * -------------------
     * This is a kind of greedy algorithm that does not always resturn a correct result.
     * See the following example where two clusters are wrongly combined into one cluster:
     *
     *                1 1 1 1
     *                 1   1 1 1 11
     *
     * y     2 2 2                   3 33 3 3 3 3
     * |      2 2 2 2 2           333 33 3 333
     * +--x
     *
     * The points are first split along the x-axis. There is however no huge gap between the
     * points along the x-axis and therefore no split is done.
     * In the second step, the points are split along the y-axis resulting in two clusters: one
     * with the 1-points and one with the 2- and 3- points (instead of 3 clusters as would be
     * expected).
     *
     * This are, however, special cases that are should be very unusual when clustering
     * obstacles on the path and even if such a situation occurs, the consequences are rather
     * unproblematic (obstacle tracking might be imperfect but the robot will not crash or
     * something like that).
     * Therefore this approximate greedy algorithm is preferred to a more complicated clustering
     * algorithm which would be computationally more expensive.
     *
     *
     * @param points List of unclustered points.
     * @param dist_threshold Minimum distance between clusters along each axis.
     * @return Partition of the points where each subset is one cluster.
     */
    static std::list<std::list<cv::Point2f> > clusterPoints(const std::list<cv::Point2f> &points,
                                                            const float dist_threshold);


private:
    struct Options : public Parameters
    {
        P<float> scale_obstacle_distance;
        P<float> scale_obstacle_lifetime;
        P<float> path_width;
        P<int> segment_step_size;
        P<float> scan_cluster_max_distance;
        P<int> min_number_of_points;

        Options():
            scale_obstacle_distance(this, "~supervisor/path_lookout/obstacle_scale_distance",  1.0f, ""),
            scale_obstacle_lifetime(this, "~supervisor/path_lookout/obstacle_scale_lifetime",  10.0f, ""),
            path_width(this, "~supervisor/path_lookout/path_width",  0.5f,
                       "Width of the path in meters (should be at least the width of the robot)."),
            segment_step_size(this, "~supervisor/path_lookout/segment_step_size",  5,
                              "Number of path segments that are approximated by a line to speed up computation."),
            scan_cluster_max_distance(this, "~supervisor/path_lookout/scan_cluster_max_distance",  0.5f,
                                      "Maximum distance between two obstacle points, to combine them to one obstacle."),
            min_number_of_points(this, "~supervisor/path_lookout/min_number_of_points",  3,
                                 "Minimum number of points on one obstacle (smaller clusters are ignored).")
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

    //! Compute weight for the given obstacle, depending on its distance to the robot and its lifetime.
    float weightObstacle(cv::Point2f robot_pos, ObstacleTracker::TrackedObstacle o) const;

    std::list<cv::Point2f> findObstaclesInCloud(const ObstacleCloud::ConstPtr &cloud);

    /**
     * @brief Cluster the given points along one axis.
     *
     * The points are sorted w.r.t. the specified axis and then partiotioned into clusters,
     * where a new cluster is started, if the distance between two neigbouring points exceeds
     * `dist_threshold`.
     *
     * The axis is defined by the function `get_value` which maps an arbitrary point to its
     * value on this axis. Example:
     *     get_x = [](const cv::Point2f&) { return p.x; };
     *
     * @param points List of points. List must not be empty!
     * @param dist_threshold
     * @param get_value Function that maps a point to its value on the desired axis.
     * @return
     */
    static std::list<std::list<cv::Point2f> > clusterPointsAlongAxis(std::list<cv::Point2f> points,
            const float dist_threshold,
            std::function<float(const cv::Point2f&)> get_value);
};

#endif // PATHLOOKOUT_H
