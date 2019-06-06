
#include "pose_writer.h"
#include <sys/time.h>
#include <fstream>
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

PoseWriter::PoseWriter()
{
    numMaxPoses_ = 2000;
}

void PoseWriter::Init(std::string parentFolder)
{
    std::ostringstream oss;

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );




    oss << (now->tm_year + 1900) <<"-"<< (now->tm_mon + 1) <<"-"<< now->tm_mday <<"_"
        << now->tm_hour <<"-"<< now->tm_min <<"-"<< now->tm_sec;

    outputFolder_ = parentFolder+"/"+oss.str();


    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    long zImgMsElapsed = (currentTime.tv_sec)*1000*1000+ currentTime.tv_usec;


    startUS_ = zImgMsElapsed;
    poseCounter_ = 0;

    int res = system(("mkdir " + outputFolder_).c_str());


}


inline std::string Pose2String(const PoseEvalResults &result)
{
    std::ostringstream oss;

    oss << result.a1 << ";";
    oss << result.a2 << ";";
    oss << result.caContactX1 << ";";
    oss << result.caContactY1 << ";";
    oss << result.caContactX2 << ";";
    oss << result.caContactY2 << ";";
    oss << result.caMinA << ";";
    oss << result.caMinB << ";";
    oss << result.cmd.x << ";";
    oss << result.cmd.y << ";";

    oss << result.deltaAngle << ";";
    oss << result.dx1 << ";";
    oss << result.dy1 << ";";
    oss << result.dx2 << ";";
    oss << result.dy2 << ";";
    oss << result.gravAngle << ";";
    oss << result.n1.x << ";";
    oss << result.n1.y << ";";
    oss << result.n1.z << ";";
    oss << result.n2.x << ";";
    oss << result.n2.y << ";";
    oss << result.n2.z << ";";
    oss << result.pose.x << ";";
    oss << result.pose.y << ";";
    oss << result.pose.z << ";";
    oss << result.poseCounter << ";";

    oss << result.stableWheelPairIdx << ";";
    oss << result.start1 << ";";
    oss << result.start2 << ";";
    oss << result.tipAngle << ";";
    oss << result.validState << ";";

    oss << result.wheelEvalResults_[0].contactPoint.x << ";";
    oss << result.wheelEvalResults_[0].contactPoint.y << ";";
    oss << result.wheelEvalResults_[0].globalWheelAngle << ";";
    oss << result.wheelEvalResults_[0].robotWheelAngle << ";";
    oss << result.wheelEvalResults_[0].wheelAngleIdx << ";";
    oss << result.wheelEvalResults_[0].wheelSupport << ";";
    oss << result.wheelEvalResults_[0].zPos << ";";
    oss << result.wheelEvalResults_[0].zValue << ";";

    oss << result.wheelEvalResults_[1].contactPoint.x << ";";
    oss << result.wheelEvalResults_[1].contactPoint.y << ";";
    oss << result.wheelEvalResults_[1].globalWheelAngle << ";";
    oss << result.wheelEvalResults_[1].robotWheelAngle << ";";
    oss << result.wheelEvalResults_[1].wheelAngleIdx << ";";
    oss << result.wheelEvalResults_[1].wheelSupport << ";";
    oss << result.wheelEvalResults_[1].zPos << ";";
    oss << result.wheelEvalResults_[1].zValue << ";";

    oss << result.wheelEvalResults_[2].contactPoint.x << ";";
    oss << result.wheelEvalResults_[2].contactPoint.y << ";";
    oss << result.wheelEvalResults_[2].globalWheelAngle << ";";
    oss << result.wheelEvalResults_[2].robotWheelAngle << ";";
    oss << result.wheelEvalResults_[2].wheelAngleIdx << ";";
    oss << result.wheelEvalResults_[2].wheelSupport << ";";
    oss << result.wheelEvalResults_[2].zPos << ";";
    oss << result.wheelEvalResults_[2].zValue << ";";

    oss << result.wheelEvalResults_[3].contactPoint.x << ";";
    oss << result.wheelEvalResults_[3].contactPoint.y << ";";
    oss << result.wheelEvalResults_[3].globalWheelAngle << ";";
    oss << result.wheelEvalResults_[3].robotWheelAngle << ";";
    oss << result.wheelEvalResults_[3].wheelAngleIdx << ";";
    oss << result.wheelEvalResults_[3].wheelSupport << ";";
    oss << result.wheelEvalResults_[3].zPos << ";";
    oss << result.wheelEvalResults_[3].zValue << ";";

    return oss.str();

}

inline std::string PoseHeader()
{
    std::ostringstream oss;

    oss << "a1"<< ";";
    oss << "a2"<< ";";
    oss << "caContactX1"<< ";";
    oss << "caContactY1"<< ";";
    oss << "caContactX2"<< ";";
    oss << "caContactY2"<< ";";
    oss << "caMinA"<< ";";
    oss << "caMinB"<< ";";
    oss << "cmd.x"<< ";";
    oss << "cmd.y"<< ";";

    oss << "deltaAngle"<< ";";
    oss << "dx1"<< ";";
    oss << "dy1"<< ";";
    oss << "dx2"<< ";";
    oss << "dy2"<< ";";
    oss << "gravAngle"<< ";";
    oss << "n1.x"<< ";";
    oss << "n1.y"<< ";";
    oss << "n1.z"<< ";";
    oss << "n2.x"<< ";";
    oss << "n2.y"<< ";";
    oss << "n2.z"<< ";";
    oss << "pose.x"<< ";";
    oss << "pose.y"<< ";";
    oss << "pose.z"<< ";";
    oss << "poseCounter"<< ";";

    oss << "stableWheelPairIdx"<< ";";
    oss << "start1"<< ";";
    oss << "start2"<< ";";
    oss << "tipAngle"<< ";";
    oss << "validState"<< ";";

    oss << "wheelEvalResults_[0].contactPoint.x"<< ";";
    oss << "wheelEvalResults_[0].contactPoint.y"<< ";";
    oss << "wheelEvalResults_[0].globalWheelAngle"<< ";";
    oss << "wheelEvalResults_[0].robotWheelAngle"<< ";";
    oss << "wheelEvalResults_[0].wheelAngleIdx"<< ";";
    oss << "wheelEvalResults_[0].wheelSupport"<< ";";
    oss << "wheelEvalResults_[0].zPos"<< ";";
    oss << "wheelEvalResults_[0].zValue"<< ";";

    oss << "wheelEvalResults_[1].contactPoint.x"<< ";";
    oss << "wheelEvalResults_[1].contactPoint.y"<< ";";
    oss << "wheelEvalResults_[1].globalWheelAngle"<< ";";
    oss << "wheelEvalResults_[1].robotWheelAngle"<< ";";
    oss << "wheelEvalResults_[1].wheelAngleIdx"<< ";";
    oss << "wheelEvalResults_[1].wheelSupport"<< ";";
    oss << "wheelEvalResults_[1].zPos"<< ";";
    oss << "wheelEvalResults_[1].zValue"<< ";";

    oss << "wheelEvalResults_[2].contactPoint.x"<< ";";
    oss << "wheelEvalResults_[2].contactPoint.y"<< ";";
    oss << "wheelEvalResults_[2].globalWheelAngle"<< ";";
    oss << "wheelEvalResults_[2].robotWheelAngle"<< ";";
    oss << "wheelEvalResults_[2].wheelAngleIdx"<< ";";
    oss << "wheelEvalResults_[2].wheelSupport"<< ";";
    oss << "wheelEvalResults_[2].zPos"<< ";";
    oss << "wheelEvalResults_[2].zValue"<< ";";

    oss << "wheelEvalResults_[3].contactPoint.x"<< ";";
    oss << "wheelEvalResults_[3].contactPoint.y"<< ";";
    oss << "wheelEvalResults_[3].globalWheelAngle"<< ";";
    oss << "wheelEvalResults_[3].robotWheelAngle"<< ";";
    oss << "wheelEvalResults_[3].wheelAngleIdx"<< ";";
    oss << "wheelEvalResults_[3].wheelSupport"<< ";";
    oss << "wheelEvalResults_[3].zPos"<< ";";
    oss << "wheelEvalResults_[3].zValue"<< ";";

    return oss.str();

}

inline std::string Pose2StringRed(const PoseEvalResults &result)
{
    std::ostringstream oss;

    oss << result.caMinA << ";";
    oss << result.caMinB << ";";
    oss << result.cmd.x << ";";
    oss << result.cmd.y << ";";

    oss << result.deltaAngle << ";";
    oss << result.gravAngle << ";";
    oss << result.n1.x << ";";
    oss << result.n1.y << ";";
    oss << result.n1.z << ";";
    oss << result.n2.x << ";";
    oss << result.n2.y << ";";
    oss << result.n2.z << ";";
    oss << result.pose.x << ";";
    oss << result.pose.y << ";";
    oss << result.pose.z << ";";
    oss << result.poseCounter << ";";

    oss << result.stableWheelPairIdx << ";";
    oss << result.tipAngle << ";";
    oss << result.validState << ";";

    oss << result.wheelEvalResults_[0].wheelSupport << ";";
    oss << result.wheelEvalResults_[0].zPos << ";";
    oss << result.wheelEvalResults_[0].zValue << ";";

    oss << result.wheelEvalResults_[1].wheelSupport << ";";
    oss << result.wheelEvalResults_[1].zPos << ";";
    oss << result.wheelEvalResults_[1].zValue << ";";

    oss << result.wheelEvalResults_[2].wheelSupport << ";";
    oss << result.wheelEvalResults_[2].zPos << ";";
    oss << result.wheelEvalResults_[2].zValue << ";";

    oss << result.wheelEvalResults_[3].wheelSupport << ";";
    oss << result.wheelEvalResults_[3].zPos << ";";
    oss << result.wheelEvalResults_[3].zValue << ";";


    return oss.str();

}

inline std::string PoseHeaderRed()
{
    std::ostringstream oss;

    oss << "caMinA"<< ";";
    oss << "caMinB"<< ";";
    oss << "cmd.x"<< ";";
    oss << "cmd.y"<< ";";

    oss << "deltaAngle"<< ";";
    oss << "gravAngle"<< ";";
    oss << "n1.x"<< ";";
    oss << "n1.y"<< ";";
    oss << "n1.z"<< ";";
    oss << "n2.x"<< ";";
    oss << "n2.y"<< ";";
    oss << "n2.z"<< ";";
    oss << "pose.x"<< ";";
    oss << "pose.y"<< ";";
    oss << "pose.z"<< ";";
    oss << "poseCounter"<< ";";

    oss << "stableWheelPairIdx"<< ";";
    oss << "tipAngle"<< ";";
    oss << "validState"<< ";";

    oss << "wheelEvalResults_[0].wheelSupport"<< ";";
    oss << "wheelEvalResults_[0].zPos"<< ";";
    oss << "wheelEvalResults_[0].zValue"<< ";";

    oss << "wheelEvalResults_[1].wheelSupport"<< ";";
    oss << "wheelEvalResults_[1].zPos"<< ";";
    oss << "wheelEvalResults_[1].zValue"<< ";";

    oss << "wheelEvalResults_[2].wheelSupport"<< ";";
    oss << "wheelEvalResults_[2].zPos"<< ";";
    oss << "wheelEvalResults_[2].zValue"<< ";";

    oss << "wheelEvalResults_[3].wheelSupport"<< ";";
    oss << "wheelEvalResults_[3].zPos"<< ";";
    oss << "wheelEvalResults_[3].zValue"<< ";";


    return oss.str();

}
void PoseWriter::WritePoses(const Trajectory *traj, const cv::Mat &dem, cv::Point2f mapOrigin)
{
    WritePoses(traj,mapOrigin);
    if (dem.rows == 0 || dem.cols == 0) return;
    std::ostringstream oss;
    oss <<outputFolder_ << "/dem"<< std::setfill('0') << std::setw(4) << poseCounter_ << ".png";
    cv::Mat writeMat;
    dem.convertTo(writeMat,CV_16U);
    cv::imwrite(oss.str(),writeMat);

}

void PoseWriter::WritePoses(const Trajectory *traj, cv::Point2f mapOrigin, cv::Point3f robotWorldPose)
{

    if (numMaxPoses_ > 0 && poseCounter_ > numMaxPoses_) return;
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    long currentUS = (currentTime.tv_sec)*1000*1000+ currentTime.tv_usec;

    long curTime = currentUS- startUS_;

    std::ostringstream oss;


    oss <<outputFolder_ << "/poseResults"<< std::setfill('0') << std::setw(4) << poseCounter_ << ".txt";


    std::ofstream fstreamPose;

    fstreamPose.open(oss.str(),std::ios::out);

    fstreamPose << curTime << ";" << robotWorldPose.x << ";" << robotWorldPose.y << ";" << robotWorldPose.z << ";" << mapOrigin.x << ";" << mapOrigin.y << ";"<<std::endl;
    fstreamPose << PoseHeaderRed() << std::endl;

    for (unsigned int tl = 0; tl < traj->poseResults_.size();tl++)
    {
        fstreamPose << Pose2StringRed(traj->poseResults_[tl]) << std::endl;

    }

    fstreamPose.close();

    poseCounter_++;

}


void WriteProcConf(std::string filename, const ProcConfig &pc,const std::string &modelFile)
{
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::WRITE);

    fs << "Proc";                              // text - mapping
    fs << "{";
    fs << "numAngleStep" << pc.numAngleStep;
    fs << "heightScale" << pc.heightScale;
    fs << "mapBaseHeight" << pc.mapBaseHeight;
    fs << "wheelGroundLevel" << pc.wheelGroundLevel;
    fs << "maxHeight" << pc.maxHeight;
    fs << "pixelSize" << pc.pixelSize;
    fs << "validThresholdFactor" << pc.validThresholdFactor;
    fs << "modelFile" << modelFile;

    fs << "}";

    fs.release();

}

void PoseWriter::WriteConfig(const ModelBasedPlannerConfig &config,const std::string &modelFile)
{
    //ProcConfig conf = config.procConfig_;
    WriteProcConf(outputFolder_ + "/procConf.txt",config.procConfig_, modelFile);

}

void PoseWriter::WriteTimings(float ms, int numPoses)
{
    std::ostringstream oss;


    oss <<outputFolder_ << "/times.txt";


    std::ofstream fstreamPose;

    fstreamPose.open(oss.str(),std::ios::out | std::ios::app);

    fstreamPose << numPoses << ";" << ms << ";"<<std::endl;

    fstreamPose.close();
}


void PoseWriter::WritePoses(const Trajectory *traj, cv::Point2f mapOrigin)
{
    if (numMaxPoses_ > 0 && poseCounter_ > numMaxPoses_) return;
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    long currentUS = (currentTime.tv_sec)*1000*1000+ currentTime.tv_usec;

    long curTime = currentUS- startUS_;

    std::ostringstream oss;


    oss <<outputFolder_ << "/poseResults"<< std::setfill('0') << std::setw(4) << poseCounter_ << ".txt";


    std::ofstream fstreamPose;

    fstreamPose.open(oss.str(),std::ios::out);

    fstreamPose << curTime << ";" << mapOrigin.x << ";" << mapOrigin.y << ";"<<std::endl;
    fstreamPose << PoseHeaderRed() << std::endl;

    for (unsigned int tl = 0; tl < traj->poseResults_.size();tl++)
    {
        fstreamPose << Pose2StringRed(traj->poseResults_[tl]) << std::endl;

    }

    fstreamPose.close();

    poseCounter_++;

}
