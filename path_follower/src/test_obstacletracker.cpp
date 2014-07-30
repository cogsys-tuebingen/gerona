#include <ros/ros.h>
#include "obstacletracker.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace std;

void prettyPrint(vector<ObstacleTracker::TrackedObstacle> obs)
{
    for (size_t i = 0; i < obs.size(); ++i) {
        cout << "(" << obs[i].last_position().x << ", " << obs[i].last_position().y << ")\t"
             << "t = " << (ros::Time::now() - obs[i].time_of_first_sight()).toNSec() << endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_obstacle_tracker");
    ros::NodeHandle nh;

    vector<cv::Point2f> obs1;
    obs1.push_back(cv::Point2f(1,1));
    obs1.push_back(cv::Point2f(2,1));
    obs1.push_back(cv::Point2f(2,2));
    obs1.push_back(cv::Point2f(3,3));
    obs1.push_back(cv::Point2f(4,1));


    vector<cv::Point2f> obs2;
    obs2.push_back(cv::Point2f(1.5,1.5));
    obs2.push_back(cv::Point2f(2,1));
    obs2.push_back(cv::Point2f(2,2.5));
    obs2.push_back(cv::Point2f(4.5,1));


    ObstacleTracker tracker;

    tracker.setMaxDist(0.6);
    tracker.update(obs1);

    cout << "Add initial" << endl;
    prettyPrint(tracker.getTrackedObstacles());

    ros::Duration(1).sleep();

    cout << "\nUpdate" << endl;
    tracker.update(obs2);
    prettyPrint(tracker.getTrackedObstacles());

    // DOES NOT WORK YET! Seems, like all tracked objects are dropped.

//    ros::Rate rate(50);

//    while(ros::ok()) {
//        ros::spinOnce();
//        rate.sleep();
//    }

    return 0;
}


