/*
 * BorderFinder.cpp
 *
 *  Created on: 15.09.2010
 *      Author: masselli
 */
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <list>
#include <execinfo.h>
#include <opencv/highgui.h>
#include "BorderFinder.h"
#include "BorderFunctions.h"

//USING_PART_OF_NAMESPACE_EIGEN
using namespace std;

BorderFinder::BorderFinder()
:
  nh_("~"),
  mIsEnabled(true),
  mDoReplan(true),
  mNewMapAvailable(false),
  mMap(NULL)
{



	mNextWaypointIndex = 0;
  //mLastReplanTime.restart();
  //mLastMapTime.restart();

    mRobotPose.px = mRobotPose.px = mRobotPose.pa = 0;
    mFrontierPose.px = mFrontierPose.py = mFrontierPose.pa = 0;
  // configuration
    nh_.param<bool>("verbose",mVerbose,true);

    // subscribe
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1,
                                                       boost::bind( &BorderFinder::globalMapCb, this, _1 ));

    // advertise
    path_pub_ = nh_.advertise<nav_msgs::Path>( "/borderpath", 5 );

    // the center is published as a point, where the z coordinate represents the circle's radius
    center_pub_ = nh_.advertise<geometry_msgs::Point> ("/arenacenter", 200);

    // read paramters from config file
    nh_.param<double>("minimum_replanh_time",mMinimumReplanTime,2);
    nh_.param<double>("minimum_replanh_time",mMinimumMapRerequestTime,2);
    nh_.param<double>("sleep_time_ms",mSleepTimeMs,20);
    nh_.param<bool>("follow_border",mFollowBorder,false);
    nh_.param<bool>("send_follow_border",mSendFollowPath,false);
    nh_.param<double>("offset_min",mOffsetMin,2.0);
    nh_.param<double>("offset_max",mOffsetMax,7.0);
    nh_.param<double>("offset_step",mOffsetStep,0.5);
    nh_.param<double>("offset_random",mOffsetRandom,0);
    nh_.param<double>("waypoint_distance",mWayPointDistance,4.0);
    nh_.param<bool>("debug_image",mShowDebugImage,false);


    mDebugWindowName = "BorderFinder";
    mExplorePath.clear();
    mCentroid.px = mCentroid.py = mCentroid.pa = 0;
    mCurrentWayPoint=mExplorePath.begin();

    if (mShowDebugImage)
        cvNamedWindow(mDebugWindowName.c_str(), 0);

}


BorderFinder::~BorderFinder() {
	if (mMap != NULL) {

    cvReleaseImage(&mMap);
    mMap = NULL;
	}
}


void BorderFinder::globalMapCb(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  if (mMap==0 || map->info.width!=mMap->width || map->info.height!=mMap->height) {
    if (mMap!=0) {
      cvReleaseImage(&mMap);
      mMap=0;
    }
    mMap= cvCreateImage(cvSize(map->info.width, map->info.height), IPL_DEPTH_8U, 1);
   }

  //Pointer to imageData
  unsigned char* dest = (unsigned char*) mMap->imageData;
  int destOffset = mMap->widthStep - mMap->width;
  //Pointer to OccupancyGridMap data
  unsigned char* source = (unsigned char*) map->data.data();
  //Iterate over image and copy data from OccupancyGridMap to IplImage
  for (unsigned int y = 0; y < map->info.height; y++) {
    for (unsigned int x = 0; x < map->info.width; x++) {
      *dest = *source;
      dest++;
      source++;
    }
    //Fix alignment
    dest += destOffset;
  }
  mNewMapAvailable = true;
}

void BorderFinder::run() {

  ros::Rate rate(200);

  while( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();
    SendPlanData();

    // send exploration target
    if (mFollowBorder) {
        SendBorderWayPoint();
    } else {
        SendPositionCommand();
    }

    if (mShowDebugImage)
        cvWaitKey(mSleepTimeMs);
    else
        usleep(mSleepTimeMs * 1000);

  }


}


void BorderFinder::FindBorder() {

	if (mMap == NULL || !mIsEnabled)
		return;


    // for runtime measurement
//	Stopwatch planningTime;
  //  Stopwatch time;

	mPath.clear();

    // amount of iterations for morphological operators
#define MORPH 2

    // erode
    //time.restart();
    cvErode(mMap, mMap, 0, MORPH);


    // floodfill from robot position
//    time.restart();
    iPoint robotPos;
    pos2mapImage(mRobotPose.px,mRobotPose.py,
                 robotPos.x, robotPos.y ,map_origin_);

    ROS_INFO_STREAM("Robot position in map coords: " << robotPos.x << "," << robotPos.y);
    // init bounding box
    iPoint min = robotPos;
    iPoint max = robotPos;

    SetFloddedCellCounter(0);
    uchar* imageArray = (uchar*)mMap->imageData;

    // look for empty space to floofill, starting from robot position
    int radius = 0;
    bool foundFillable =    floodfill(imageArray, robotPos.x,        robotPos.y, &max, &min, mMap->width, mMap->height);

    while ( !foundFillable  && (radius < 20) ) {
        foundFillable =     floodfill(imageArray, robotPos.x+radius, robotPos.y, &max, &min, mMap->width, mMap->height);
        if (!foundFillable)
            foundFillable = floodfill(imageArray, robotPos.x, robotPos.y+radius, &max, &min, mMap->width, mMap->height);
        if (!foundFillable)
            foundFillable = floodfill(imageArray, robotPos.x-radius, robotPos.y, &max, &min, mMap->width, mMap->height);
        if (!foundFillable)
            foundFillable = floodfill(imageArray, robotPos.x, robotPos.y-radius, &max, &min, mMap->width, mMap->height);

        ++radius;
    }


    // dilate
    cvDilate(mMap, mMap, 0, MORPH);


    // for debugging
    imageArray[robotPos.x + robotPos.y*mMap->width] = CELL_CONTOUR;
    imageArray[robotPos.x+1 + robotPos.y*mMap->width] = CELL_CONTOUR;
    imageArray[robotPos.x-1 + robotPos.y*mMap->width] = CELL_CONTOUR;
    imageArray[robotPos.x + (robotPos.y+1)*mMap->width] = CELL_CONTOUR;
    imageArray[robotPos.x + (robotPos.y-1)*mMap->width] = CELL_CONTOUR;

    // follow contour

    if ( (min.x < max.x) && (min.y < max.y) ) { // if there's a bounding box
        // resize it
        min.x -= (MORPH+1);
        min.y -= (MORPH+1);
        max.x += (MORPH+1);
        max.y += (MORPH+1);
    }
    iPoint contour[MAXCONTOURLENGTH];
    bool occupied[MAXCONTOURLENGTH];

    int contourLength = contourFollow(imageArray, contour,
                                      min.x, min.y, max.x, max.y,
                                      mMap->width, mMap->height,
                                      occupied);


//    VOUT("Exploration rate " << unexploredCounter / sqrt(gFloddedCellCounter));
        
    if (contourLength > 0) {
        // get convex hull
        iPoint deque[2*MAXCONTOURLENGTH + 1];
        iPoint* convexHull;
        float border[MAXCONTOURLENGTH];
        int convexHullLength = convexHull2D(contour, contourLength, deque, &convexHull);
        ROS_INFO_STREAM("Length of convex hull: " << convexHullLength);

        // close path
        convexHull[convexHullLength] = convexHull[0];
        ++convexHullLength;

        // classify
        classifyContour(contour, contourLength, convexHull, convexHullLength, occupied, border);
        float frontierRatio;
        iPoint frontier = findLargestFrontier(contour, contourLength, occupied, &frontierRatio);

        ROS_INFO_STREAM("Frontier ratio: " << frontierRatio);
        border[0] = frontierRatio; // so planner knows the ratio

        // copy to mPath
        for (int i = 0; i < convexHullLength; ++i) {
            pose2d_t node = mapImage2pos(convexHull[i].pose2d(),map_origin_);
//            OUT("border: " << border[i]);
            node.pa = border[i];
            mPath.push_back(node);
        }

        mFrontierPose = mapImage2pos(frontier.pose2d(),map_origin_);
        mFrontierPose.pa = frontierRatio; // abuse pa to send frontier ratio

        if (mVerbose) {
            pose2d_t node = mFrontierPose;
            node.pa = -1;
            mPath.push_back(node);
        }

        mNextWaypointIndex = 0;
    }

    if (mPath.size() == 0 )
        ROS_INFO_STREAM("BorderFinder: Could not find a valid border.");

    // Update states
    mDoReplan = false;


    // Display map
    if (mShowDebugImage) {        
        cvShowImage(mDebugWindowName.c_str(), mMap);
    }
    if (mFollowBorder && (mPath.size()>4)) {
        CalcCentroid();
        CalcInnerPolygon(mOffsetMin,mOffsetMax,mOffsetStep,mOffsetRandom,mExplorePath);
        mCurrentWayPoint=mExplorePath.begin();
    } else {
    }
}
bool BorderFinder::getRobotPose(pose2d_t& pose, const std::string& map_frame )
{
    tf::StampedTransform trafo;
    geometry_msgs::TransformStamped msg;
    try {
        tf_.lookupTransform( map_frame.c_str(), "/base_link", ros::Time(0), trafo );
    } catch (tf::TransformException& ex) {
        ROS_ERROR( "Error getting the robot position. Reason: %s",
                   ex.what());
        return false;
    }

    tf::transformStampedTFToMsg( trafo, msg );
    pose.px = msg.transform.translation.x;
    pose.py = msg.transform.translation.y;
    pose.pa = tf::getYaw( msg.transform.rotation );

    return true;
}


void BorderFinder::pos2mapImage(double px, double py, int& mx, int&my, const geometry_msgs::Pose& map_origin)
{

}


void BorderFinder::mapImage2pos(double mx, double my, double& px, double&py, const geometry_msgs::Pose& map_origin)
{

}


pose2d_t BorderFinder::map2pos(const pose2d_t& p, const geometry_msgs::Pose& map_origin)
{

}

pose2d_t BorderFinder::mapImage2pos(const pose2d_t& p,const geometry_msgs::Pose& map_origin)
{

}



/*
void BorderFinder::GetMap() {

  //Copy map...
	player_map_data_t* mapdata = (player_map_data_t*)msg->GetPayload();
	uchar* imageArray = (uchar*) mMap->imageData +
			mMap->widthStep * (mMap->height - 1); // We start in the last row
	int lineOffset = - (mMap->width * mMap->nChannels + mMap->widthStep);
	char* mapArray = (char*) mapdata->data;

	for (int y = 0; y < mMap->height; y++) {
		for (int x = 0; x < mMap->width; x++) {
			//cout << " " << int(mapdata->data[y * mapdata->width + x]);
            if (*mapArray < 0)  // empty   //== -mapdata->data_range
                *imageArray = CELL_EMPTY;
            else if (*mapArray > 0) // occupied // == mapdata->data_range
                *imageArray = CELL_OCCUPIED;
			else
                *imageArray = CELL_UNEXPLORED; // unknown
			imageArray++;
			mapArray++;
		}
		imageArray += lineOffset;
	}

	mDoReplan = true;
}
*/


void BorderFinder::SendWaypoints() {


/*
    if (mSendFollowPath) {
        reply.waypoints_count=mExplorePath.size();

    } else {
        reply.waypoints_count = mPath.size();
    }
  reply.waypoints = (pose2d_t*)calloc(sizeof(reply.waypoints[0]),reply.waypoints_count);
	int index = 0;
    if (mSendFollowPath) {

        for(Vector2dList::iterator it = mExplorePath.begin(); it != mExplorePath.end(); ++it){
            reply.waypoints[index].px = it->x();
            reply.waypoints[index].py = it->y();
            reply.waypoints[index].pa = 0;
            ++index;
        }
    } else {
        for(vector<pose2d_t>::iterator it = mPath.begin(); it != mPath.end(); ++it){
            //cout << index << ": " << (*i).px << endl;
            reply.waypoints[index].px = it->px;
            reply.waypoints[index].py = it->py;
            reply.waypoints[index].pa = it->pa;
            ++index;
        }
    }
    this->Publish(mBorderAddress, resp_queue,
    */
}

void BorderFinder::SendPlanData() {
 //   VOUT("SendPlanData()");
        WriteContourData();
        /*
      int pathSize=0;
      if (mSendFollowPath) {
          pathSize=mExplorePath.size();
      } else {
          pathSize = mPath.size();
      }
      if(pathSize < 1) {
        plannerData.valid = 0;
        plannerData.done = 0;
	  }
      else if(mNextWaypointIndex >= pathSize) {
          plannerData.done = 1;
          plannerData.valid = 1;
	  }
	  else {
          plannerData.done = 0;
          plannerData.valid = 1;
	  }
	  // put the current localize pose
      plannerData.pos = mRobotPose;

      if(plannerData.valid) {
        plannerData.waypoint = mPath[mNextWaypointIndex];
        plannerData.waypoint_idx = mNextWaypointIndex;
        plannerData.waypoints_count = pathSize;
	  }

      this->Publish(mBorderAddress,
	                PLAYER_MSGTYPE_DATA,
	                PLAYER_PLANNER_DATA_STATE,
                    (void*)&plannerData, sizeof(plannerData), NULL);
	  mLastPlanPublishTime.restart();
    */
}

void BorderFinder::WriteContourData ()
{
    if (!mVerbose) return;

    std::ofstream of;
    of.open("/tmp/contour.dat", std::ios::out | std::ios::app);
    of << "% new countour" << std::endl;
    of << "% x y" << std::endl;
    for (int i=0;i< mPath.size();++i) {
        of << mPath[i].px << " "<< mPath[i].py << std::endl;
    }
    of << "% end of contour"<< endl;
    of.close();
}

void BorderFinder::CalcInnerPolygon (double offsetMin, double offsetMax, double offsetStep,
                                     double offsetRandom, Vector2dList &polygon)
{
    if (mPath.size()<2) {
        return;
    }
    // find index of nearest point on path to robot pos
    Vector2d P;
    P.x() = mRobotPose.px;
    P.y() = mRobotPose.py;
    int nearestIdx = -1;
    double dist = 1e10;
    int pathSize = mPath.size();
    for (int i=0;i<pathSize;++i) {
        Vector2d B;
        B.x()=mPath[i].px;
        B.y()=mPath[i].py;
        Vector2d PB = B-P;
        double d=PB.norm();
        if (d<dist) {
            nearestIdx=i;
            dist = d;
        }
    }
    ROS_INFO_STREAM("nearest point at x="<<mPath[nearestIdx].px << "y="<<mPath[nearestIdx].py);
    ROS_INFO_STREAM("robotposx="<<P.x()<<" y="<<P.y());
    polygon.clear();
    Vector2d CB,CBL;
    Vector2d C;
    C.x()=mCentroid.px;
    C.y()=mCentroid.py;
    // create polygon paths through the field
    Vector2d Pprev;
    Pprev.x() = mRobotPose.px;
    Pprev.y() = mRobotPose.py;

    // disthresh determines how far away points shoudl be
    double distThresh = mWayPointDistance;
    // start with nearest point on boundary
    int idx = nearestIdx;
    int cnt=0;
    while (cnt<pathSize) {
        if (idx>=pathSize || idx<0) {
            idx = 0;
        }
        Vector2d B,Q;
        // point B on boundary
        B.x()=mPath[idx].px;
        B.y()=mPath[idx].py;
        CB = B-C;
        double l1=CB.norm();

        CB.normalize();
        double offset = offsetMin+(random()%2)*mOffsetRandom;

        double l=l1-offset;
        if (l>0) {
            // Q is point on path
            Q = C+CB*l;
            Vector2d PpQ=Q-Pprev;
            if (PpQ.norm()>distThresh) {
                while (l>l1-offsetMax) {
                    iPoint m;
                    pos2mapImage(Q.x(),Q.y(),m.x,m.y,map_origin_);
                    uchar state=GetMapState(m.x,m.y);

                    if (state!=CELL_OCCUPIED) {
                        polygon.push_back(Q);
                        Pprev=Q;
                        break;
                    }
                    l=l-offsetStep;
                    if (l<0) {
                        break;
                    }
                    Q = C+CB*l;
                }
            }
        }
        // next point on border
        ++idx;
        ++cnt;
    }
    if (polygon.size()>=1) {
        Vector2d F=polygon.front();
        polygon.push_back(F);
    }
}


uchar BorderFinder::GetMapState(int i, int j)
{
    if (i<0 || j<0 ||i>= mMap->width || j>=mMap->height) {
        return CELL_OCCUPIED;
    }
    int width=mMap->width;
    return mMap->imageData[i*width+j];
}


void BorderFinder::CalcCentroid ()
{
    // calc area of polygon first
    double area = 0;
    for (int i =0;i<mPath.size()-1;++i) {
        double& xi=mPath[i].px;
        double& yi=mPath[i].py;
        double& xin=mPath[i+1].px;
        double& yin=mPath[i+1].py;
        area = area+xi*yin-xin*yi;
    }
    area = area*0.5;
    if (area<=0.0) {
        mCentroid.px = 0;
        mCentroid.py = 0;
    } else {
        // calc center coordinates
        double cx=0, cy=0;
        for (int i =0;i<mPath.size()-1;++i) {
            double& xi=mPath[i].px;
            double& yi=mPath[i].py;
            double& xin=mPath[i+1].px;
            double& yin=mPath[i+1].py;
            cx = cx+(xi+xin)*(xi*yin-xin*yi);
            cy = cy+(yi+yin)*(xi*yin-xin*yi);
        }
        mCentroid.px=cx/(6.0*area);
        mCentroid.py=cy/(6.0*area);
        ROS_INFO_STREAM("cx cy"<<mCentroid.px<< " "<<mCentroid.py);
    }
}




void BorderFinder::SendPositionCommand() {

    pose2d_t position = mFrontierPose;
/*
    Publish(mTargetPositionAddress, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,
            (void*)&position, sizeof(position), NULL);*/
}

void BorderFinder::SendBorderWayPoint()
{
    pose2d_t position;
    if (mExplorePath.size()<3) {
        position = mFrontierPose;
    } else {

        Vector2d P;
        P.x() = mRobotPose.px;
        P.y() = mRobotPose.py;
        if (mCurrentWayPoint == mExplorePath.end()) {
            ROS_INFO_STREAM("end reached");
            mCurrentWayPoint = mExplorePath.begin();
        }
        Vector2d W=*mCurrentWayPoint;
        Vector2d WP=P-W;
        double dist = WP.norm();
        if (dist<3.0) {
            ++mCurrentWayPoint;
            if (mCurrentWayPoint == mExplorePath.end()) {
                mCurrentWayPoint = mExplorePath.begin();
            }
            W=*mCurrentWayPoint;
        }
        position.px=W.x();
        position.py=W.y();
        position.pa=0;
    }
    // ***todo publish target
//    Publish(mTargetPositionAddress, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,
  //          (void*)&position, sizeof(position), NULL);
}


int main(int argc, char** argv)
{
  ros::init(argc,argv, "border_finder");

  BorderFinder border_finder;

  border_finder.run();
  return 0;
}
