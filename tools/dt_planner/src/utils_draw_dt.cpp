
#include <utils_draw_dt.h>
#include "plannertraj_dt.h"
#include "poseevalresults_dt.h"
/*
void DrawProc::DrawRobot(ScaledDrawProc &drawProc,IModelBasedPlanner &planner)
{
    //PlannerBase* bplanner = std::static_pointer_cast<PlannerBase>(IModelBasedPlanner);





}
*/


cv::Mat DrawProc::D16SImageToRGB(cv::Mat image, int imin, int imax)
{
    cv::Mat displayImg;

    image.convertTo(displayImg,CV_8U,256.0/(double)(imax-imin),-imin*256.0/(double)(imax-imin));

    cv::cvtColor(displayImg,displayImg,CV_GRAY2BGR);

    return displayImg;

}


void DrawProc::DrawMapStates(const cv::Mat &dem, cv::Mat &drawMat, const ProcConfigDT &config)
{
    for (int yl = 0; yl < dem.rows;++yl)
    {
        for (int xl = 0; xl < dem.cols;++xl)
        {
            if (dem.at<short>(yl,xl) == 0) drawMat.at<cv::Vec3b>(yl,xl) = cv::Vec3b(0,0,128);
            if (dem.at<short>(yl,xl) == config.notVisibleLevel) drawMat.at<cv::Vec3b>(yl,xl) = cv::Vec3b(0,128,128);

        }

    }



}

cv::Mat DrawProc::Draw(ScaledDrawProc &sdp,const cv::Mat dem, const ProcConfigDT &config,std::vector<TrajNodeDT*> &trajectories, const TrajectoryDT* bestTraj_, cv::Point3f goal,cv::Point3f robotPos)
{
    //drawZMin_ = - config.mapBaseHeight/40;
    //drawZMax_ =  config.mapBaseHeight/40;


    cv::Mat drawMat = D16SImageToRGB(dem,config.mapBaseHeight+ drawZMin_,config.mapBaseHeight+drawZMax_);

    DrawMapStates(dem,drawMat,config);




    SetupDrawProc(sdp,drawMat,drawScaleFactor_);

    DrawTrajectories(sdp,trajectories,bestTraj_);


    DrawGoal(sdp,goal, robotPos);


    return sdp.GetImage();

}


void DrawProc::DrawGoal(ScaledDrawProc &drawProc,cv::Point3f goal,cv::Point3f robotPos)
{


    drawProc.DrawLineScaled(cv::Point2f(robotPos.x,robotPos.y),cv::Point2f(goal.x,goal.y),cv::Scalar(200,124,10),2);

}

void DrawProc::DrawPath(ScaledDrawProc &drawProc,std::vector<cv::Point3f> path)
{
    if (path.empty()) return;

    for (unsigned int tl = 1; tl < path.size();++tl)
    {
        cv::Point2f p1;
        p1.x = path[tl-1].x;
        p1.y = path[tl-1].y;

        cv::Point2f p2;
        p2.x = path[tl].x;
        p2.y = path[tl].y;

        drawProc.DrawLineScaled(p1,p2,cv::Scalar(10,124,200),2);

    }

}



void DrawProc::SetupDrawProc(ScaledDrawProc &drawProc,cv::Mat &img, float scaleFactor)
{

    drawProc.SetScaleFactor(scaleFactor);

    cv::Mat tres,resultsmall;

    if (img.type() != CV_8UC3)
    {
        img.convertTo(tres,CV_8U);
        cv::cvtColor(tres,resultsmall,CV_GRAY2BGR);

    }
    else
    {
        resultsmall = img;
    }

    drawProc.SetMat(resultsmall);

}

cv::Scalar DrawProc::GetEndStateColor(int validState)
{
    switch (validState)
    {


    case PERSDT_GOALREACHED:         return cv::Scalar(150,255,150);
    case PERSDT_VALID:               return cv::Scalar(0,255,0);

    case PERSDT_COLLISION: return cv::Scalar(255,255,0);


    case PERSDT_NOTASSIGNED:         return cv::Scalar(50,50,50);
    case PERSDT_OUTOFIMAGE:          return cv::Scalar(155,155,155);

    default :                      return cv::Scalar(255,255,255);
    }

}

void DrawProc::DrawTrajectories(ScaledDrawProc &sdp, const std::vector<TrajNodeDT*> &trajectories, const TrajectoryDT* bestTraj_)
{
    if (trajectories.size() == 0) return;
    for (unsigned int tl = 0; tl < trajectories.size();++tl)
    {
        TrajNodeDT* curT = trajectories[tl];

        PoseEvalResultsDT* curRes;
        PoseEvalResultsDT* curRes2;

        for (unsigned int i = 0; i < curT->poseResults_.size();++i)
        {
            curRes = &curT->poseResults_[i];
            if (curRes->validState == PERSDT_NOTASSIGNED) break;

            float gAngle = (curRes->minDist / (conf.procConfig_.pixelSizeInv*0.5));

            if (conf.scorerConfig_.distanceThreshold > 0) gAngle = (curRes->minDist / (conf.scorerConfig_.distanceThresholdImg*3.0));

            float maxAngle = std::min(gAngle,1.0f);
            float resAngle = maxAngle;


            //cv::Scalar color(0 ,(int)(255- (resAngle/2.0)*255), (int)((resAngle/2.0)*255));
            cv::Scalar color((int)(255.0- (resAngle)*255.0), (int)((resAngle)*255.0) , 0);

            //float tgravAngle = (maxAngle-0.95)*(1.0/0.05);



            if (curRes->validState >= 0)sdp.DrawCrossScaled(cv::Point2f(curRes->pose.x,curRes->pose.y),1,color);
            else
            {
                sdp.DrawCrossScaled(cv::Point2f(curRes->pose.x,curRes->pose.y),1,cv::Scalar(0,255.0,255.0));
            }

            if (i +1 < curT->poseResults_.size())
            {
                curRes2 = &curT->poseResults_[i+1];
                if (curRes2->validState != PERSDT_NOTASSIGNED)
                {
                    sdp.DrawLineScaled(cv::Point2f(curRes->pose.x,curRes->pose.y),cv::Point2f(curRes2->pose.x,curRes2->pose.y),color);

                }
            }

        }


        if (curT->start_ != nullptr && curT->poseResults_.size() > 0)
        {
            curRes = &curT->poseResults_[0];
            if (curRes->validState >= 0 && curT->start_->validState >= 0) sdp.DrawLineScaled(cv::Point2f(curT->start_->pose.x,curT->start_->pose.y),cv::Point2f(curRes->pose.x,curRes->pose.y),cv::Scalar(0,255.0,0));
            else sdp.DrawLineScaled(cv::Point2f(curT->start_->pose.x,curT->start_->pose.y),cv::Point2f(curRes->pose.x,curRes->pose.y),cv::Scalar(0,255.0,255.0));

        }

        /*
        if (curT->poseResults_.size() > 1)
        {



            for (unsigned int i = 0; i < curT->poseResults_.size()-1;++i)
            {
                curRes = &curT->poseResults_[i];
                if (curRes->validState < 0) break;

                curRes2 = &curT->poseResults_[i+1];
                if (curRes2->validState < 0) break;

                float tgravAngle = std::abs(curRes->gravAngle-curRes2->gravAngle);
                if (tgravAngle < 0) tgravAngle = 0;
                if (curRes->validState >= 0 && curRes2->validState >= 0) sdp.DrawLineScaled(cv::Point2f(curRes->pose.x,curRes->pose.y),cv::Point2f(curRes2->pose.x,curRes2->pose.y),cv::Scalar(0,255.0*(1.0-tgravAngle),255.0*(tgravAngle)));
                else sdp.DrawLineScaled(cv::Point2f(curRes->pose.x,curRes->pose.y),cv::Point2f(curRes2->pose.x,curRes2->pose.y),cv::Scalar(0,255.0,255.0));

            }
        }
        */


        if (curT->end_ != nullptr)sdp.DrawCircleScaled(cv::Point2f(curT->end_->pose.x,curT->end_->pose.y),3,GetEndStateColor(curT->end_->validState),2);



    }

    if (bestTraj_ != nullptr)
    {
        const PoseEvalResultsDT* curRes;

        for (unsigned int i = 0; i < bestTraj_->poseResults_.size();++i)
        {

            curRes = &bestTraj_->poseResults_[i];
            if (curRes->validState >= 0)sdp.DrawCircleScaled(cv::Point2f(curRes->pose.x,curRes->pose.y),2,cv::Scalar(255.0,0,255.0));

        }

    }


}


cv::Mat DrawProc::GenerateMask(const cv::Mat &img)
{
    cv::Mat result(img.size(),CV_8U);

    const short *imgPtr;
    unsigned char *maskPtr;


    for (int y = 0; y < img.rows;++y)
    {
        imgPtr= img.ptr<short>(y);
        maskPtr= result.ptr<unsigned char >(y);


        for (int x = 0; x < img.cols;++x)
        {
            //if (imgPtr[x] >= conf.procConfig_.maxHeight) maskPtr[x] = 0;
            //else maskPtr[x] = 1;
        }
    }

    return result;

}




void DrawProc::DrawRobotScaled(ScaledDrawProc &drawProc, RobotModelDT &model, PoseEvalResultsDT &results)
{
    const cv::Point2f pos(results.pose.x,results.pose.y);
    const float angle = results.pose.z;



    ProcConfigDT procConfig = model.GetProcConfig();

    int idx = model.GetAngleIdx(angle);



    const RobotDescriptorDT rDesc = model.GetDescriptor(idx);



    cv::Point2f curPos = pos-rDesc.baseLinkPosImage_;


    drawProc.DrawCrossScaled(pos,3,cv::Scalar(0,0,255));


//    DrawRobotWheelScaled(drawProc,model.GetWheel(0),curPos+rDesc.wheelPositionsImage_[0],waIdx0,results.wheelEvalResults_[0].contactPoint);
//    DrawRobotWheelScaled(drawProc,model.GetWheel(1),curPos+rDesc.wheelPositionsImage_[1],waIdx1,results.wheelEvalResults_[1].contactPoint);
//    DrawRobotWheelScaled(drawProc,model.GetWheel(2),curPos+rDesc.wheelPositionsImage_[2],waIdx2,results.wheelEvalResults_[2].contactPoint);
//    DrawRobotWheelScaled(drawProc,model.GetWheel(3),curPos+rDesc.wheelPositionsImage_[3],waIdx3,results.wheelEvalResults_[3].contactPoint);

    //drawProc.PutText(curPos+rDesc.wheelPositionsImage_[0]*1.2,"0",cv::Scalar(255,0,255));
    //drawProc.PutText(curPos+rDesc.wheelPositionsImage_[1]*1.2,"1",cv::Scalar(255,0,255));
    //drawProc.PutText(curPos+rDesc.wheelPositionsImage_[2]*1.2,"2",cv::Scalar(255,0,255));
    //drawProc.PutText(curPos+rDesc.wheelPositionsImage_[3]*1.2,"3",cv::Scalar(255,0,255));


    drawProc.DrawLineScaled(pos,curPos,cv::Scalar(0,0,255));

    int numPoints = model.GetNumTestPoints();

    for (int i = 0; i < numPoints;++i)
    {
        TestPointConfig tp = model.GetTestPoint(i);
        drawProc.DrawLineScaled(curPos+rDesc.testPositionsImage_[i],curPos,cv::Scalar(0,255,0));

        cv::Scalar cColor(0,255,0);
        if (results.distances[i] <= 0) cColor = cv::Scalar(255,0,0);

        drawProc.DrawCircleScaled(curPos+rDesc.testPositionsImage_[i],tp.radiusImg,cColor,1);
        if (results.distances[i] > 0 && results.distances[i] < 1000)drawProc.DrawCircleScaled(curPos+rDesc.testPositionsImage_[i],results.distances[i],cv::Scalar(0,255,128),1);

    }


    //cv::Point2f n1(results.n1.x,-results.n1.y);
    //cv::Point2f n2(results.n2.x,-results.n2.y);


    //drawProc.DrawLineScaled(curPos+n1*100.0,curPos,cv::Scalar(0,255,255));
    //drawProc.DrawLineScaled(curPos+n2*100.0,curPos,cv::Scalar(255,255,0));





    //DrawWheel(image,rgbTemp,mask8u,pos,pos+tpose->bl_,pos+tpose->blTP_);
    //DrawWheel(image,rgbTemp,mask8u,pos,pos+tpose->br_,pos+tpose->brTP_);
    //DrawWheel(image,rgbTemp,mask8u,pos,pos+tpose->fr_,pos+tpose->frTP_);
    //DrawWheel(image,rgbTemp,mask8u,pos,pos+tpose->fl_,pos+tpose->flTP_);

    //cv::line(image,cv::Point2f(pos.x,pos.y),cv::Point2f(pos.x,pos.y)-(cv::Point2f(norm1.x,norm1.y)*30.0),cv::Scalar(0,0,255));
    //cv::line(image,cv::Point2f(pos.x,pos.y),cv::Point2f(pos.x,pos.y)-(cv::Point2f(norm2.x,norm2.y)*30.0),cv::Scalar(0,255,0));



    //DrawWheelLines(image,pos+tpose->bl_,wAngles[0],distances[0],valid);
    //DrawWheelLines(image,pos+tpose->br_,wAngles[1],distances[1],valid);
    //DrawWheelLines(image,pos+tpose->fr_,wAngles[2],distances[2],valid);
    //DrawWheelLines(image,pos+tpose->fl_,wAngles[3],distances[3],valid);




    //cv::circle(result,pos,3,cv::Scalar(0,0,255));

    //drawProc.DrawCrossScaled(curPos+rDesc.gravCenterImage_,3,cv::Scalar(255,0,255));


    //return drawProc.GetImage();


}

