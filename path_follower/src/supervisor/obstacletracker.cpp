#include "obstacletracker.h"

#include <boost/foreach.hpp>
#include <algorithm>

using namespace std;

ObstacleTracker::ObstacleTracker()
{
    ros::param::param<float>("~obstacle_tracker/max_dist", opt_.max_dist_, 0.3f);
    int ll;
    ros::param::param<int>("~obstacle_tracker/lost_lifetime", ll, 1.0f);
    opt_.lost_lifetime_ = ros::Duration(ll);
}

void ObstacleTracker::update(std::vector<Obstacle> observed_obstacles)
{
    /**
     * There is a slight difference in the actual implementation of the algorithmn, compared to the description in the
     * class comment:
     * Since removing rows/columns from the distance matrix changes the indices of the remaining entries, it would be
     * quite complicated to keep the connetion between the obstacles and the entries in the matrix. Therefore, instead
     * of cutting the distance matrix ('dist'), an additional matrix 'd_idx' is constructed, containing indices to the
     * fields in the distance matrix. This index-matrix is then used to cut away matched obstacles, instead of the
     * distance matrix itself.
     */

    //FIXME: exploit enclosing circle of obstacles for matching (match if center is within the circle or something like that)


    // Delete "dead" obstacles, which could not be matched for more than the time, specified in lost_lifetime_.
    // Use ugly c++ version of functional "filter"...
    // TODO: I think it's not the best solution to drop old entries before matching, as this will completely crush
    // any tracking, if lost_lifetime ist set to 0 ("do not track obstacles that are out of sight"). Doing it after
    // matching will cause the same problem though, as there is a small amount of time passing between the two steps.
    // It has somehow to be guaranteed, that no obstacles are dropped, that could be matched in this iteration.
    size_t dead_count = obstacles_.size();
    obstacles_.erase(std::remove_if(obstacles_.begin(), obstacles_.end(), boost::bind(&ObstacleTracker::isDead, this, _1)), obstacles_.end());
    dead_count = dead_count - obstacles_.size();

    // construct distanc matrix. D[i,j] = distance between observed obstacle i and tracked obstacle j.
    cv::Mat dist(observed_obstacles.size(), obstacles_.size(), CV_32F);
    cv::Mat d_idx(dist.rows, dist.cols, CV_16U);
    vector<bool> obs_matched(observed_obstacles.size(), false);

    ushort i = 0; // ATTENTION: i is the data-index to dist. Therefore the outer loop must iterate over rows, the inner over columns!
    for (size_t o = 0; o < observed_obstacles.size(); ++o)
    for (size_t t = 0; t < obstacles_.size();         ++t) {
        dist.at<float>(o, t)   = norm(observed_obstacles[o].center - obstacles_[t].obstacle().center);
        d_idx.at<ushort>(o, t) = i++;

    }

    //ROS_DEBUG_STREAM("D =\n" << dist);

    // now match the points
    int match_counter = 0;
    while (!d_idx.empty()) {
        //ROS_DEBUG_STREAM("Didx =\n" << d_idx);

        // min_idx = argmin(D)
        float min = dist.at<float>(d_idx.at<ushort>(0));
        int min_idx = 0;
        for (int i = 1; i < d_idx.rows*d_idx.cols; ++i) {
            int idx = d_idx.at<ushort>(i);
            if (dist.at<float>(idx) < min) {
                min = dist.at<float>(idx);
                min_idx = i;
            }
        }

        //ROS_DEBUG("min idx = %d (%d), dist = %g", d_idx.at<ushort>(min_idx), min_idx, dist.at<float>(d_idx.at<ushort>(min_idx)));

        // If the shortest distance is to big, assume that all matchable obstacles are matched.
        if (min > opt_.max_dist_) {
            break;
        }

        // data-index to distance-matrix coords (mi, mj are the indices of the matched obstacles)
        int mi = d_idx.at<ushort>(min_idx) / dist.cols;
        int mj = d_idx.at<ushort>(min_idx) % dist.cols;
        // ... and to index-matrix coords (those are only needed, to remove the corresponding row and column, a few lines below)
        int mi_idx = min_idx / d_idx.cols;
        int mj_idx = min_idx % d_idx.cols;

        //ROS_DEBUG("matched %d, %d (%d,%d)", mi, mj, mi_idx, mj_idx);

        // match observed obstacle mi with tracked obstacle mj
        obstacles_[mj].update(observed_obstacles[mi]);
        ++match_counter;
        // mark mi as matched.
        obs_matched[mi] = true;

        // delete mi and mj from the distances index matrix
        removeRow(d_idx, mi_idx);
        if (d_idx.empty())
            break; // stop before calling removeColumn on an empty matrix will cause a crash.
        removeColumn(d_idx, mj_idx);
    }

    ROS_DEBUG_NAMED("ObstacleTracker", "Matched %d obstacles", match_counter);
    ROS_DEBUG_NAMED("ObstacleTracker", "Lost %zu obstacles", obstacles_.size() - match_counter);
    ROS_DEBUG_NAMED("ObstacleTracker", "Droped %zu dead obstacles", dead_count);

    // If there are unmatched obstacles in the observation, add them as new obstacles
    int add_counter = 0;
    for (size_t i = 0; i < observed_obstacles.size(); ++i) {
        if (!obs_matched[i]) {
            TrackedObstacle to(observed_obstacles[i]);
            obstacles_.push_back(to);
            ++add_counter;
        }
    }

    ROS_DEBUG_NAMED("ObstacleTracker", "Added %d new obstacles", add_counter);
}

void ObstacleTracker::reset()
{
    obstacles_.clear();
}

bool ObstacleTracker::isDead(ObstacleTracker::TrackedObstacle o)
{
    return (ros::Time::now() - o.time_of_last_sight()) > opt_.lost_lifetime_;
}

void ObstacleTracker::removeRow(cv::Mat &m, int row)
{
    assert(row >= 0 && row < m.rows);

    // {v|h}concat can not handle empty matrices, thus there are some nasty case distinctions necessary...
    // Beware: end row/col of range-methods is exclusive!
    if (row == 0) {
        m = m.rowRange(1,m.rows);
    } else if (row == m.rows-1) {
        m = m.rowRange(0,row);
    } else {
        cv::vconcat(m.rowRange(0,row), m.rowRange(row+1, m.rows),  m);
    }
}

void ObstacleTracker::removeColumn(cv::Mat &m, int col)
{
    assert(col >= 0 && col < m.cols);

    // {v|h}concat can not handle empty matrices, thus there are some nasty case distinctions necessary...
    // Beware: end col/col of range-methods is exclusive!
    if (col == 0) {
        m = m.colRange(1,m.cols);
    } else if (col == m.cols-1) {
        m = m.colRange(0,col);
    } else {
        cv::hconcat(m.colRange(0,col), m.colRange(col+1, m.cols),  m);
    }
}
