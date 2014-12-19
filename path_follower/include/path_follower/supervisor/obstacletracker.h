#ifndef OBSTACLETRACKER_H
#define OBSTACLETRACKER_H

/// STL
#include <vector>

/// THIRD PARTY
#include <opencv2/imgproc/imgproc.hpp>

/// ROS
#include <ros/ros.h>

/// PROJECT
#include <path_follower/utils/maptransformer.h>
#include <path_follower/utils/parameters.h>
#include <path_msgs/Obstacle.h>

struct Obstacle
{
    // Obstacle is representat as circle.
    cv::Point2f center;
    float radius;

    path_msgs::Obstacle toMsg() const
    {
        path_msgs::Obstacle msg;
        msg.position.x = center.x;
        msg.position.y = center.y;
        msg.position.z = 0;
        msg.radius = radius;

        return msg;
    }
};

/**
 * @brief Track obstacles on the path.
 *
 * The tracking algorithm (implemented in update()) is quite simple:
 * Input: New observation (= list of detected obstacles)
 *   - Try to match observed obstacles with those that are already tracked (see below, how matching is done).
 *   - Drop tracked obstacles which had no match for a defined duration.
 *   - Add observed obstacles w/o matching partner in the list of currently tracked obstacles as new.
 *
 * Matching:
 *   1) Construct a matrix with all distances between tracked and observed obstacles (observed = rows, tracked = columns)
 *   2) Search the matrix for the pair with the minimal distance. Match this pair.
 *   3) Remove the row and the column, related to the matched obstacles, from the distance matrix.
 *   4) Iterate until distance matrix is empty (= all are matched), or the minimal distance in the remaining matrix is
 *      greater than a defined threshold (-> do not match, if they are to far away from each other).
 *
 * Currently only the center point of an obstacle is used.
 */
class ObstacleTracker
{
public:
    class TrackedObstacle {
    public:
        TrackedObstacle(Obstacle obs):
            obstacle_(obs),
            time_of_first_sight_(ros::Time::now()),
            time_of_last_sight_(ros::Time::now()),
            id_(nextId())
        {}

        void update(Obstacle obs)
        {
            obstacle_ = obs;
            time_of_last_sight_ = ros::Time::now();
        }

        Obstacle obstacle() const
        {
            return obstacle_;
        }

        ros::Time time_of_first_sight() const
        {
            return time_of_first_sight_;
        }

        ros::Time time_of_last_sight() const
        {
            return time_of_last_sight_;
        }

        int id() const
        {
            return id_;
        }

    private:
        //! Last known position of the obstacle.
        Obstacle obstacle_;
        //! Time, when the obstacle was detected for the first time (i.e. when it has been added to the list of tracked obstacles).
        ros::Time time_of_first_sight_;
        //! Time, when the obstacle was detected for the last time (to track lost obstacles).
        ros::Time time_of_last_sight_;
        //! Unique identifier
        int id_;

        static int nextId()
        {
            static int x = 0;
            return x++;
        }
    };


    void setMaxDist(float md)
    {
        opt_.max_dist.set(md);
    }

    std::vector<TrackedObstacle> getTrackedObstacles() const
    {
        return obstacles_;
    }


    /**
     * @brief Update the list ob tracked obstacles with a new observation.
     * @param obstacles List of obstacles observed in the last scan.
     */
    void update(std::vector<Obstacle> obstacles);

    //! Reset the tracker (= drop all tracked obstacles).
    void reset();

private:
    struct Options: public Parameters
    {
        P<float> max_dist;
        P<float> lost_lifetime;

        Options():
            max_dist(this, "~obstacle_tracker/max_dist", 0.3f, "Only match new observation with tracked obstacle, if the position change is less than this threshold."),
            lost_lifetime(this, "~obstacle_tracker/lost_lifetime", 1.0f, "Duration (in seconds) for which a lost obstacle is still tracked at its last known position.")
        {}
    } opt_;

    //! List of tracked obstacles
    std::vector<TrackedObstacle> obstacles_;

    //! Check if an obstacle is dead (= no observation for more than the allowed ''lost lifetime'') and thus can be
    //! removed from the list.
    bool isDead(TrackedObstacle o);

    /**
     * @brief Remove a row from a matrix.
     * @param m    The matrix.
     * @param row  Row-Index.
     */
    static inline void removeRow(cv::Mat &m, int row);

    /**
     * @brief Remove a column from a matrix.
     * @param m    The matrix.
     * @param col  Column-Index.
     */
    static inline void removeColumn(cv::Mat &m, int col);
};

#endif // OBSTACLETRACKER_H
