/*
 * BorderFinder.h
 *
 *  Created on: 15.09.2010
 *      Author: masselli
 */

#ifndef BORDERFINDER_H_
#define BORDERFINDER_H_
#include <ros/ros.h>

#include <Eigen/Core>
#include <string>
#include <opencv/cv.h>
#include <vector>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "Path2d.h"


#define CELL_OCCUPIED     100
#define CELL_UNEXPLORED 50
#define CELL_EMPTY      0
#define CELL_FILLED     255
#define CELL_CONTOUR     64

#define MAXCONTOURLENGTH 4096


struct iPoint {
    pose2d_t pose2d() {
        pose2d_t res;
        res.px = x; res.py = y; res.pa = 0;
        return res;
    }

    int x;
    int y;
};
USING_PART_OF_NAMESPACE_EIGEN

struct BorderWayPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector2d pos;
    int occ;  // 0 = occupied, 255 = free & reachable
};
typedef std::list<BorderWayPoint> BorderWayPointList;
typedef std::vector<BorderWayPoint> BorderWayPointVec;

typedef std::list<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Vector2dList;

class BorderFinder
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BorderFinder();
    virtual ~BorderFinder();

    bool getRobotPose( pose2d_t &pose, const std::string& map_frame );

    void globalMapCb(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void run();

private:
    void FindBorder();
    void WriteContourData ();
    void CalcCentroid ();
    /**
      calc polygon for exploring from boundary
      @param minOffset min dist from boundary
      @þaram maxOffset max dist from boundary
    */
    void CalcInnerPolygon (double minOffset,double maxOffset, double offsetStep,
                           double offsetRandom, Vector2dList& polygon);
    void SendWaypoints();
    void SetState();
    void SendPositionCommand();
    void SendBorderWayPoint();
    void SendPlanData();

    void pos2mapImage(double px, double py, int& mx, int&my, const geometry_msgs::Pose& map_origin);
    void mapImage2pos(double mx, double my, double& px, double&py, const geometry_msgs::Pose& map_origin);
    pose2d_t pos2map(const pose2d_t& p, const geometry_msgs::Pose& map_origin);
    pose2d_t pos2mapImage(const pose2d_t& p, const geometry_msgs::Pose& map_origin);

    pose2d_t map2pos(const pose2d_t& p, const geometry_msgs::Pose& map_origin);
    pose2d_t mapImage2pos(const pose2d_t& p,const geometry_msgs::Pose& map_origin);

    uchar GetMapState(int i, int j);

    ros::Subscriber map_sub_;
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Publisher center_pub_;
    tf::TransformListener tf_;
    geometry_msgs::Pose map_origin_;



    double mSleepTimeMs; // in milliseconds
    IplImage* mMap;
    Path2d mPath;
    pose2d_t mCentroid;
    bool    mSendFollowPath;
    bool    mFollowBorder;
    // list of exploration points
    Vector2dList mExplorePath;
    double      mOffsetMin,mOffsetMax,mOffsetStep, mOffsetRandom;
    double      mWayPointDistance;
    Vector2dList::iterator mCurrentWayPoint;
    int mNextWaypointIndex;
    pose2d_t mRobotPose;
    pose2d_t mFrontierPose;
    double mMinimumReplanTime;
    bool mDoReplan;
    bool mIsEnabled;
    bool mNewMapAvailable;
    bool mReachedGoal;
    double mMinimumMapRerequestTime;
    bool mShowDebugImage;
    //Stopwatch mLastReplanTime;
    //Stopwatch mLastMapTime;
    //Stopwatch mLastPlanPublishTime;
    bool mVerbose;

    std::string mDebugWindowName;
};
#endif // BORDERFINDER_H_
