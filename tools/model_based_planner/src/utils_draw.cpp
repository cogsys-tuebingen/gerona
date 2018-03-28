
#include <utils_draw.h>
#include "plannertraj.h"
#include "utils_diff.h"

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


void DrawProc::DrawMapStates(const cv::Mat &dem, cv::Mat &drawMat, const ProcConfig &config)
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

cv::Mat DrawProc::Draw(ScaledDrawProc &sdp,const cv::Mat dem, const ProcConfig &config,std::vector<TrajNode*> &trajectories, const Trajectory* bestTraj_, cv::Point3f goal,cv::Point3f robotPos)
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


    drawProc.DrawLineScaled(cv::Point2f(robotPos.x,robotPos.y),cv::Point2f(goal.x,goal.y),cv::Scalar(200,124,10),3);

}

void DrawProc::DrawPath(ScaledDrawProc &drawProc,std::vector<cv::Point3f> path)
{
    if (path.empty()) return;

    for (int tl = 1; tl < path.size();++tl)
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
    case PERS_EXCEEDDELTAANGLE:    return cv::Scalar(100,10,255);
    case PERS_EXCEEDGRAVANGLE:     return cv::Scalar(10,100,255);
    case PERS_EXCEEDTIPANGLE:      return cv::Scalar(100,100,255);
    case PERS_GOALREACHED:         return cv::Scalar(10,255,100);
    case PERS_NOTVISIBLE:          return cv::Scalar(100,255,10);

    case PERS_LOWWHEELSUPPORT:     return cv::Scalar(255,10,10);
    case PERS_NOWHEELSUPPORT:      return cv::Scalar(255,100,100);

    case PERS_NOTASSIGNED:         return cv::Scalar(50,50,50);
    case PERS_OUTOFIMAGE:          return cv::Scalar(155,155,155);
    case PERS_SAFE_OUTOFIMAGE:     return cv::Scalar(155,155,155);
    case PERS_VALID:               return cv::Scalar(0,255,0);
    default :                       return cv::Scalar(255,255,255);
    }

}

void DrawProc::DrawTrajectories(ScaledDrawProc &sdp, const std::vector<TrajNode*> &trajectories, const Trajectory* bestTraj_)
{
    if (trajectories.size() == 0) return;
    for (unsigned int tl = 0; tl < trajectories.size();++tl)
    {
        TrajNode* curT = trajectories[tl];

        PoseEvalResults* curRes;
        PoseEvalResults* curRes2;

        for (unsigned int i = 0; i < curT->poseResults_.size();++i)
        {
            curRes = &curT->poseResults_[i];
            if (curRes->validState == PERS_NOTASSIGNED) break;

            float gAngle = (curRes->gravAngle / conf.scorerConfig_.gravAngleThreshold);
            float tAngle = (curRes->tipAngle / conf.scorerConfig_.tipAngleThreshold);
            float dAngle = (curRes->deltaAngle / conf.scorerConfig_.deltaAngleThreshold);

            int curAngle = 0;
            float maxAngle = std::min(gAngle,1.0f);
            float resAngle = maxAngle;
            //cv::Scalar color(1.0f-((maxAngle/2.0 + 0.5))*255 ,(maxAngle/2.0 + 0.5)*255,255);
            if (tAngle > maxAngle)
            {
                maxAngle = tAngle;
                curAngle = 1;
                resAngle = std::min(tAngle,1.0f);
                //color = cv::Scalar((maxAngle/2.0 + 0.5)*255 ,0,255);
            }
            if (dAngle > maxAngle)
            {
                maxAngle = dAngle;
                curAngle = 2;
                resAngle = std::min(dAngle,1.0f);
                //color = cv::Scalar(0 ,(maxAngle/2.0 + 0.5)*255,255);
            }

            //cv::Scalar color(0 ,(int)(255- (resAngle/2.0)*255), (int)((resAngle/2.0)*255));
            cv::Scalar color(0 ,(int)(255- (resAngle)*255), (int)((resAngle)*255));
            if (curAngle == 1) color = cv::Scalar ((int)((resAngle)*255) ,(int)(255- (resAngle)*255), 0);
            if (curAngle == 2) color = cv::Scalar (128 ,(int)(255- (resAngle)*255), (int)((resAngle)*255));


            //float tgravAngle = (maxAngle-0.95)*(1.0/0.05);



            if (curRes->validState >= 0)sdp.DrawCrossScaled(cv::Point2f(curRes->pose.x,curRes->pose.y),1,color);
            else
            {
                sdp.DrawCrossScaled(cv::Point2f(curRes->pose.x,curRes->pose.y),1,cv::Scalar(0,255.0,255.0));
            }

            if (i +1 < curT->poseResults_.size())
            {
                curRes2 = &curT->poseResults_[i+1];
                if (curRes2->validState != PERS_NOTASSIGNED)
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
        const PoseEvalResults* curRes;

        for (unsigned int i = 0; i < bestTraj_->poseResults_.size();++i)
        {

            curRes = &bestTraj_->poseResults_[i];
            if (curRes->validState >= 0)sdp.DrawCircleScaled(cv::Point2f(curRes->pose.x,curRes->pose.y),2,cv::Scalar(255.0,0,255.0));

        }

    }


}

void DrawProc::DrawWheelLinesScaled(ScaledDrawProc &proc, cv::Point2f &origin, WheelModel &model, int angle)
{
    WheelDescriptor desc = model.GetDescriptorIdx(angle);

    cv::Point2f xline = desc.dirX_*model.radiusImg_;
    cv::Point2f yline = desc.dirY_*(model.widthImg_/2.0f);


    cv::Point2f center = origin+desc.centerImg_;

    //center*=scaleFactor;

    proc.DrawLineScaled(center+xline+yline,center-xline+yline,cv::Scalar(255,0,0));
    proc.DrawLineScaled(center+xline-yline,center-xline-yline,cv::Scalar(255,0,0));
    proc.DrawLineScaled(center+xline+yline,center+xline-yline,cv::Scalar(255,0,0));
    proc.DrawLineScaled(center-xline+yline,center-xline-yline,cv::Scalar(255,0,0));


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
            if (imgPtr[x] >= conf.procConfig_.maxHeight) maskPtr[x] = 0;
            else maskPtr[x] = 1;
        }
    }

    return result;

}

void DrawProc::DrawRobotWheelScaled(ScaledDrawProc &proc, WheelModel &model, cv::Point2f pos, int angleIdx,cv::Point2i contact)
{
    WheelDescriptor desc = model.GetDescriptorIdx(angleIdx);
    cv::Point2f imageOrigin = (pos- desc.jointPosImg_);

    cv::Mat wheelImage;
    cv::Mat colorImg;

    desc.image_->mat_.convertTo(wheelImage,CV_8U,1.0,-conf.procConfig_.wheelGroundLevel);

    cv::cvtColor(wheelImage,colorImg,CV_GRAY2BGR);


    cv::Mat mask = GenerateMask(desc.image_->mat_);
    proc.DrawImage(colorImg,mask,imageOrigin);


    proc.DrawLineScaled(imageOrigin+desc.centerImg_,pos,cv::Scalar(250,0,0));


    DrawWheelLinesScaled(proc, imageOrigin, model, angleIdx);

    if (contact.x > 0) proc.DrawCircleScaled(imageOrigin+cv::Point2f(contact.x,contact.y),1,cv::Scalar(0,0,250));


}


void DrawProc::DrawChassis(ScaledDrawProc &proc, ProcConfig &procConfig, ChassisModel &model, PoseEvalResults &results, cv::Point2f pos, int angleIdx)
{

    ChassisDescriptor desc = model.GetDescriptorIdx(angleIdx);

    cv::Point2f imageOrigin = (pos- desc.centerImg_);



    cv::Mat chassisImage;
    cv::Mat colorImg;


    CVAlignedMat::ptr tempChassis = CVAlignedMat::Create(desc.image_->mat_);

    float cStart = results.start1;
    float cDX = results.dx1;
    float cDY = results.dy1;
    float cCoX = results.caContactX1;
    float cCoY = results.caContactY1;
    float caMin = results.caMinA;

    if (results.a2 < results.a1)
    {
        cStart = results.start2;
        cDX = results.dx2;
        cDY = results.dy2;
        cCoX = results.caContactX2;
        cCoY = results.caContactY2;
        caMin = results.caMinB;


    }


    Utils_DIFF::warpChassis(desc.image_->mat_,tempChassis->mat_,cStart,cDX,cDY);

    ///Debug
    short minVal = 30000, maxVal = -30000;
    short *chMatPtr;
    for (int y = 0; y < desc.image_->mat_.rows;++y)
    {
        chMatPtr = desc.image_->mat_.ptr<short>(y);
        for (int x = 0; x < desc.image_->mat_.cols;++x)
        {
            if (chMatPtr[x] < procConfig.maxHeight)
            {
                if (chMatPtr[x] < minVal) minVal = chMatPtr[x];
                if (chMatPtr[x] > maxVal) maxVal = chMatPtr[x];
            }
        }
    }


    //float tChassisOffset = cStart;

    tempChassis->mat_.convertTo(chassisImage,CV_8U,1.0,-procConfig.wheelGroundLevel);

    cv::cvtColor(chassisImage,colorImg,CV_GRAY2BGR);



    cv::Mat mask = GenerateMask(desc.image_->mat_);
    proc.DrawImage(colorImg,mask,imageOrigin);

    proc.DrawCrossScaled(pos,2,cv::Scalar(0,0,200));

    proc.DrawCrossScaled(imageOrigin,2,cv::Scalar(0,200,0));

    if (caMin-procConfig.mapBaseHeight < 0)proc.DrawCrossScaled(imageOrigin+cv::Point2f(cCoX,cCoY),2,cv::Scalar(200,0,200));




}


void DrawProc::DrawRobotScaled(ScaledDrawProc &drawProc, RobotModel &model, PoseEvalResults &results)
{
    const cv::Point2f pos(results.pose.x,results.pose.y);
    const float angle = results.pose.z;



    ProcConfig procConfig = model.GetProcConfig();

    int idx = model.GetAngleIdx(angle);



    const RobotDescriptor rDesc = model.GetDescriptor(idx);



    cv::Point2f curPos = pos-rDesc.baseLinkPosImage_;

    if (model.GetChassis().TestChassis())
    {
        DrawChassis(drawProc,procConfig,model.GetChassis(),results,curPos+rDesc.chassisPosImage_,idx);



    }

    drawProc.DrawCrossScaled(pos,3,cv::Scalar(0,0,255));


    int waIdx0 = results.wheelEvalResults_[0].wheelAngleIdx;
    int waIdx1 = results.wheelEvalResults_[1].wheelAngleIdx;
    int waIdx2 = results.wheelEvalResults_[2].wheelAngleIdx;
    int waIdx3 = results.wheelEvalResults_[3].wheelAngleIdx;




    DrawRobotWheelScaled(drawProc,model.GetWheel(0),curPos+rDesc.wheelPositionsImage_[0],waIdx0,results.wheelEvalResults_[0].contactPoint);
    DrawRobotWheelScaled(drawProc,model.GetWheel(1),curPos+rDesc.wheelPositionsImage_[1],waIdx1,results.wheelEvalResults_[1].contactPoint);
    DrawRobotWheelScaled(drawProc,model.GetWheel(2),curPos+rDesc.wheelPositionsImage_[2],waIdx2,results.wheelEvalResults_[2].contactPoint);
    DrawRobotWheelScaled(drawProc,model.GetWheel(3),curPos+rDesc.wheelPositionsImage_[3],waIdx3,results.wheelEvalResults_[3].contactPoint);

    //drawProc.PutText(curPos+rDesc.wheelPositionsImage_[0]*1.2,"0",cv::Scalar(255,0,255));
    //drawProc.PutText(curPos+rDesc.wheelPositionsImage_[1]*1.2,"1",cv::Scalar(255,0,255));
    //drawProc.PutText(curPos+rDesc.wheelPositionsImage_[2]*1.2,"2",cv::Scalar(255,0,255));
    //drawProc.PutText(curPos+rDesc.wheelPositionsImage_[3]*1.2,"3",cv::Scalar(255,0,255));


    drawProc.DrawLineScaled(pos,curPos,cv::Scalar(0,0,255));

    drawProc.DrawLineScaled(curPos+rDesc.wheelPositionsImage_[0],curPos,cv::Scalar(0,255,0));
    drawProc.DrawLineScaled(curPos+rDesc.wheelPositionsImage_[1],curPos,cv::Scalar(0,255,0));
    drawProc.DrawLineScaled(curPos+rDesc.wheelPositionsImage_[2],curPos,cv::Scalar(0,255,0));
    drawProc.DrawLineScaled(curPos+rDesc.wheelPositionsImage_[3],curPos,cv::Scalar(0,255,0));


    cv::Point2f n1(results.n1.x,-results.n1.y);
    cv::Point2f n2(results.n2.x,-results.n2.y);


    drawProc.DrawLineScaled(curPos+n1*100.0,curPos,cv::Scalar(0,255,255));
    drawProc.DrawLineScaled(curPos+n2*100.0,curPos,cv::Scalar(255,255,0));





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

