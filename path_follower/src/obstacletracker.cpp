#include "obstacletracker.h"

#include <boost/foreach.hpp>

using namespace std;

void ObstacleTracker::update(std::vector<cv::Point2f> observed_obstacles)
{
    // construct distanc matrix. D[i,j] = distance between observed obstacle i and tracked obstacle j.
    cv::Mat dist(observed_obstacles.size(), obstacles_.size(), CV_32F);
    cv::Mat d_idx(dist.rows, dist.cols, CV_16U);
    vector<bool> obs_matched(observed_obstacles.size(), false);

    ushort i = 0;
    for (size_t o = 0; o < observed_obstacles.size(); ++o)
    for (size_t t = 0; t < obstacles_.size();         ++t) {
        dist.at<float>(o, t)   = norm(observed_obstacles[o] - obstacles_[t].last_position);
        d_idx.at<ushort>(o, t) = i++; // ATTENTION: This is the data-index to dist. It will only work, if this nested loop
                                      //            iterates over rows first!
    }

    ROS_INFO_STREAM("D = " << dist);

    // now match the points
    vector<TrackedObstacle> matched_tracked;

    while (!d_idx.empty()) {
        ROS_INFO_STREAM("Didx = " << d_idx);

        // min_idx = argmin(D)
        float min = dist.at<float>(d_idx.data[0]);
        int min_idx = 0;
        for (int i = 1; i < d_idx.rows*d_idx.cols; ++i) {
            int idx = d_idx.data[i];
            if (dist.at<float>(idx) < min) {
                min = dist.at<float>(idx);
                min_idx = idx;
            }
        }

        ROS_INFO("min idx = %d", min_idx);

        // If the shortest distance is to big, assume that all matchable obstacles are matched.
        if (min > max_dist_) {
            break;
        }

        // data-index to matrix coords
        int mi = min_idx / dist.cols;
        int mj = min_idx % dist.cols;

        ROS_INFO("matched %d, %d", mi, mj);

        // match observed obstacle mi with tracked obstacle mj
        obstacles_[mj].last_position = observed_obstacles[mi];
        matched_tracked.push_back(obstacles_[mj]);
        // mark mi as matched.
        obs_matched[mi] = true;

        // delete mi and mj from the distances index matrix
        //FIXME: test, what happens, if range is 0..-1 (i.e. mi or mj == 0 or == last) --> Problems!
        cv::vconcat(d_idx.rowRange(0,mi-1), d_idx.rowRange(mi+1, d_idx.rows-1),  d_idx);
        cv::hconcat(d_idx.colRange(0,mj-1), d_idx.colRange(mj+1, d_idx.cols-1),  d_idx);
    }

    // Delete tracked obstacles, which could not be matched (assume that they left the path). This is done by
    // overwriting the old list of tracked obstacles by the list of matched tracked obstacles.
    obstacles_ = matched_tracked;

    // If there are unmatched obstacles in the observation, add them as new obstacles
    for (size_t i = 0; i < observed_obstacles.size(); ++i) {
        if (!obs_matched[i]) {
            TrackedObstacle to;
            to.last_position = observed_obstacles[i];
            to.time_of_first_sight = ros::Time::now();

            obstacles_.push_back(to);
        }
    }
}
