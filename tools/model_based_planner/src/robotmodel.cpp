#include "robotmodel.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "utils_math_approx.h"


RobotModel::RobotModel()
{

}

/*
int RobotModel::EvaluateWheel(const cv::Mat &dem,const int wheelIdx, const cv::Point2f &pos, const cv::Point2f &cmd, const float &robotAngle, const int &robotAngleIdx, WheelEvalResults &results) const
{
    const WheelModel &wm = wheels_[wheelIdx];
    results.angle = GetAngle(wm,cmd,robotAngle);
    results.angleIdx = GetWheelAngleIdx(results.angle,robotAngleIdx,wm);
    return wm.Evaluate(dem,pos,results);


}
*/

float RobotModel::GetBaseLinkZ(const PoseEvalResults &results)
{

    float startZ = 0;
    cv::Point2f wPos(0,0);
    const int angleIdx = GetAngleIdx(results.pose.z);

    const RobotDescriptor rDesc = GetDescriptor(angleIdx);


    if (results.stableWheelPairIdx == 0)
    {
        startZ = results.wheelEvalResults_[0].zValue;
        wPos = rDesc.wheelPositionsImage_[0];
    }
    else
    {
        startZ = results.wheelEvalResults_[1].zValue;
        wPos = rDesc.wheelPositionsImage_[1];
    }


    cv::Point2f blPos = rDesc.baseLinkPosImage_;

    cv::Point2f w2bl = blPos-wPos;

    float resZ = (float)startZ- w2bl.x * results.dx1 + w2bl.y * results.dy1;

    const float z0 = -(resZ-procConfig_.mapBaseHeight)*procConfig_.heightScaleInv;


    return z0;

    //wPos


}


int RobotModel::EvaluatePose(const cv::Mat &dem, PoseEvalResults &results) const
{
    /// Get robot descriptor for pose
    const cv::Point2f pos(results.pose.x,results.pose.y);
    const float angle = NormalizeAngle(results.pose.z);
    const int angleIdx = GetAngleIdxFast(angle);
    const RobotDescriptor &desc = GetDescriptor(angleIdx);
    const cv::Point2f robotCenter = pos-desc.baseLinkPosImage_;


    /// Get wheel models
    const WheelModel &wm0 = wheels_[0];
    const WheelModel &wm1 = wheels_[1];
    const WheelModel &wm2 = wheels_[2];
    const WheelModel &wm3 = wheels_[3];


    /// Set the current orienation of wheels
    SetWheelAngle(angle,angleIdx,wm0,results.wheelEvalResults_[0]);
    SetWheelAngle(angle,angleIdx,wm1,results.wheelEvalResults_[1]);
    SetWheelAngle(angle,angleIdx,wm2,results.wheelEvalResults_[2]);
    SetWheelAngle(angle,angleIdx,wm3,results.wheelEvalResults_[3]);

    //int iz0,iz1,iz2,iz3;

    /// Evaluate all four wheels
    const int wr0 = wm0.Evaluate(dem,robotCenter+desc.wheelPositionsImage_[0],results.wheelEvalResults_[0]);
    const int wr1 = wm1.Evaluate(dem,robotCenter+desc.wheelPositionsImage_[1],results.wheelEvalResults_[1]);
    const int wr2 = wm2.Evaluate(dem,robotCenter+desc.wheelPositionsImage_[2],results.wheelEvalResults_[2]);
    const int wr3 = wm3.Evaluate(dem,robotCenter+desc.wheelPositionsImage_[3],results.wheelEvalResults_[3]);

    /// if one wheel is invalid, return outofimage
    if (wr0 == -1 || wr1 == -1 || wr2 == -1 || wr3 == -1)
    {
        results.validState = PERS_OUTOFIMAGE;
        return PERS_OUTOFIMAGE;
    }

    /// Get wheel z-positions
    const float z0 = -(float)(results.wheelEvalResults_[0].zValue-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;
    const float z1 = -(float)(results.wheelEvalResults_[1].zValue-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;
    const float z2 = -(float)(results.wheelEvalResults_[2].zValue-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;
    const float z3 = -(float)(results.wheelEvalResults_[3].zValue-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;


    const float d01 = z0-z1;
    //const float d03 = z0-z3;
    //const float d12 = z1-z2;
    const float d23 = z2-z3;

    // z0-z1 - z3+z2
    // = -z3 + z2 - z1 + z0
    const float conf0 = lw2_4*d01 + lw1_4*d23;


    float nAx,nAy,nBx,nBy;

    //cv::Point3f pA,pB;

    int cw1 = 0;
    int cw2 = 0;


    /// Calc vehicle normals
    if (conf0 < 0)
    {// calc normal for wheel 0 & 2
        nAx = -w1_2*z3+w1mw2*z1+w1pw2*z0;
        nAy = -l_2*d01;
        nBx = w1mw2*z3+mw1mw2*z2+w2_2*z1;
        nBy = l_2*d23;
        //results.centerZ = (z1+z3)/2.0;

        cw1 = 0;
        cw2 = 2;

        // Wheels 0 and 2 are unstable
        // Wheels 1 and 3 are stable
        results.stableWheelPairIdx = 1;

        /*
        pA.x = desc.wheelPositionsImage_[0].x;
        pA.y = desc.wheelPositionsImage_[0].y;
        pA.z = z0;

        pB.x = desc.wheelPositionsImage_[2].x;
        pB.y = desc.wheelPositionsImage_[2].y;
        pB.z = z2;
        */



    }
    else
    { // calc normal for wheel 1 & 3
        nAx = -w1_2*z2+w1pw2*z1+w1mw2*z0;
        nAy = -l_2*d01;
        nBx = mw1mw2*z3+w1mw2*z2+w2_2*z0;
        nBy = l_2*d23;
        //results.centerZ = (z0+z2)/2.0;

        cw1 = 1;
        cw2 = 3;
        // Wheels 1 and 3 are unstable
        // Wheels 0 and 2 are stable
        results.stableWheelPairIdx = 0;

        /*
        pA.x = desc.wheelPositionsImage_[1].x;
        pA.y = desc.wheelPositionsImage_[1].y;
        pA.z = z1;

        pB.x = desc.wheelPositionsImage_[3].x;
        pB.y = desc.wheelPositionsImage_[3].y;
        pB.z = z3;
        */


    }
    //const float lA = 1.0/sqrt(nAx*nAx+nAy*nAy+lw1_4Sqr);
    //const float lB = 1.0/sqrt(nBx*nBx+nBy*nBy+lw2_4Sqr);
    const float lA = Utils_Math_Approx::frsqrt(nAx*nAx+nAy*nAy+lw1_4Sqr);
    const float lB = Utils_Math_Approx::frsqrt(nBx*nBx+nBy*nBy+lw2_4Sqr);

    const float n1x = nAx*lA;
    const float n1y = nAy*lA;

    const float n2x = nBx*lB;
    const float n2y = nBy*lB;

    results.nr1.x = n1x;
    results.nr1.y = -n1y;
    results.nr1.z = lw1_4*lA;


    results.nr2.x = n2x;
    results.nr2.y = -n2y;
    results.nr2.z = lw2_4*lB;



    results.n1.x = desc.cosa_*n1x + desc.sina_*n1y;
    results.n1.y = (desc.sina_*n1x - desc.cosa_*n1y);
    results.n1.z = results.nr1.z;


    results.n2.x = desc.cosa_*n2x + desc.sina_*n2y;
    results.n2.y = (desc.sina_*n2x - desc.cosa_*n2y);
    results.n2.z = results.nr2.z;

    const float agA = Utils_Math_Approx::facos(lw1_4*lA);
    const float agB = Utils_Math_Approx::facos(lw2_4*lB);

    results.a1 = agA;
    results.a2 = agB;

    results.tipAngle = Utils_Math_Approx::facos( results.n1.dot(results.n2) );
    results.gravAngle = (agA>agB?agA:agB);


    results.wheelEvalResults_[0].zPos = z0;
    results.wheelEvalResults_[1].zPos = z1;
    results.wheelEvalResults_[2].zPos = z2;
    results.wheelEvalResults_[3].zPos = z3;

    //const float absZInv1 = results.n1.z == 0 ? 0 : 1.0f/std::abs(results.n1.z);
    //const float absZInv2 = results.n2.z == 0 ? 0 : 1.0f/std::abs(results.n2.z);
    const float absZInv1 = results.n1.z == 0 ? 0 : 1.0f/(results.n1.z);
    const float absZInv2 = results.n2.z == 0 ? 0 : 1.0f/(results.n2.z);

    const float tn1x = desc.cosa_*n1x + desc.sina_*n1y;
    const float tn1y = desc.sina_*n1x - desc.cosa_*n1y;
    const float tn2x = desc.cosa_*n2x + desc.sina_*n2y;
    const float tn2y = desc.sina_*n2x - desc.cosa_*n2y;

    const float dx1 = -( tn1x*absZInv1)*procConfig_.heightPixelRatio;
    const float dy1 = -( tn1y*absZInv1)*procConfig_.heightPixelRatio;

    const float dx2 = -( tn2x*absZInv2)*procConfig_.heightPixelRatio;
    const float dy2 = -( tn2y*absZInv2)*procConfig_.heightPixelRatio;

    results.dx1 = dx1;
    results.dy1 = dy1;
    results.dx2 = dx2;
    results.dy2 = dy2;

    const cv::Point2f w1  = desc.wheelPositionsImage_[cw1];//+wheels_[cw1].descriptors_[angleIdx].jointPosImg_;
    const cv::Point2f w2  = desc.wheelPositionsImage_[cw2];//+wheels_[cw2].descriptors_[angleIdx].jointPosImg_;

    cv::Point2f blPos = desc.baseLinkPosImage_;

    cv::Point2f w12bl = w1-blPos;
    cv::Point2f w22bl = w2-blPos;

    results.z1 = (float)results.wheelEvalResults_[cw1].zPos - (w12bl.x * dx1 + w12bl.y * dy1)*procConfig_.heightScaleInv;
    results.z2 = (float)results.wheelEvalResults_[cw2].zPos - (w22bl.x * dx2 + w22bl.y * dy2)*procConfig_.heightScaleInv;

    //results.z1 = (results.z1-procConfig_.mapBaseHeight)*procConfig_.heightScaleInv;
    //results.z2 = (results.z2-procConfig_.mapBaseHeight)*procConfig_.heightScaleInv;

    results.r1 = Utils_Math_Approx::fasin(n1y);//*0.7;
    results.p1 = Utils_Math_Approx::fasin(n1x);//*0.7;
    results.r2 = Utils_Math_Approx::fasin(n2y);//*0.7;
    results.p2 = Utils_Math_Approx::fasin(n2x);//*0.7;



    results.validState = PERS_VALID;

    /// Get Test if chassis testing is required.
    if (chassisModel_.TestChassis())
    {        

        const ChassisDescriptor chassisDesc = chassisModel_.GetDescriptorIdx(angleIdx);

        const cv::Point2f chassisPos = desc.chassisPosImage_ - chassisDesc.centerImg_;


        const cv::Point2f  wcPos1 = w1 - (chassisPos);
        const cv::Point2f  wcPos2 = w2 - (chassisPos);


        //const float startValA = (results.wheelEvalResults_[cw1].zPos*procConfig_.heightScale)- wcPos1.x*dx1 - wcPos1.y*dy1;
        //const float startValB = (results.wheelEvalResults_[cw2].zPos*procConfig_.heightScale)- wcPos2.x*dx2 - wcPos2.y*dy2;

        /// Get the start height for both configurations
        const float startValA = (results.wheelEvalResults_[cw1].zPos*procConfig_.heightScale)- wcPos1.x*dx1 - wcPos1.y*dy1;
        const float startValB = (results.wheelEvalResults_[cw2].zPos*procConfig_.heightScale)- wcPos2.x*dx2 - wcPos2.y*dy2;


        int cx1 = 0;
        int cy1 = 0;

        ////TESTING
        /*
        //float testW1 = startValA + dx1*wcPos1.x + dy1*wcPos1.y;
        //float testW2 = startValB + dx2*wcPos2.x + dy2*wcPos2.y;

        int twIdx3 = (cw1+1)%4;
        const cv::Point2f w3  = desc.wheelPositionsImage_[twIdx3];//+wheels_[2].descriptors_[angleIdx].jointPosImg_;

        int twIdx4 = (cw2+1)%4;
        const cv::Point2f w4  = desc.wheelPositionsImage_[twIdx4];//+wheels_[2].descriptors_[angleIdx].jointPosImg_;

        const cv::Point2f  wcPos3 =w3-(chassisPos);
        const cv::Point2f  wcPos4 =w4-(chassisPos);


        //const cv::Point2f  wcPos31 = w3-w1;
        //const cv::Point2f  wcPos32 = w3-w2;

        //float test13 = (results.wheelEvalResults_[cw1].zPos*procConfig_.heightScale) + dx1*wcPos31.x + dy1*wcPos31.y;
        //float test23 = (results.wheelEvalResults_[cw2].zPos*procConfig_.heightScale) + dx2*wcPos32.x + dy2*wcPos32.y;
        //float testZC = startValB + dx2*wcPos3.x + dy2*wcPos3.y;

        float test3a = startValA + dx1*wcPos3.x + dy1*wcPos3.y;
        float test3b = startValB + dx2*wcPos3.x + dy2*wcPos3.y;

        float test4a = startValA + dx1*wcPos4.x + dy1*wcPos4.y;
        float test4b = startValB + dx2*wcPos4.x + dy2*wcPos4.y;


        float eps = 0.001;

        if (cw1 == 0)
        {
            if ( std::abs(test3a-z1*procConfig_.heightScale) > eps ||
                 std::abs(test3b-z1*procConfig_.heightScale) > eps ||
                 std::abs(test4a-z3*procConfig_.heightScale) > eps ||
                 std::abs(test4b-z3*procConfig_.heightScale) > eps)
            {
                cx1 = 1;
            }
        }

        if (cw1 == 1)
        {
            if ( std::abs(test3a-z2*procConfig_.heightScale) > eps ||
                 std::abs(test3b-z2*procConfig_.heightScale) > eps ||
                 std::abs(test4a-z0*procConfig_.heightScale) > eps ||
                 std::abs(test4b-z0*procConfig_.heightScale) > eps)
            {
                cy1 = 2;
            }
        }
        */
        ////TESTING


        results.start1 = startValA;
        results.start2 = startValB;



        /// Test first configuration, the second one is only tested if the angle between both is larger than the threshold
        int chassisTestA = chassisModel_.EvaluateNP(dem,startValA,dx1,dy1,robotCenter+desc.chassisPosImage_,angleIdx,cx1,cy1);

        int cx2 = 0;
        int cy2 = 0;
        int chassisTestB = chassisTestA;
        if (results.tipAngle < config_.chassisTestTipAngleThreshold)
        {
            chassisTestB = chassisModel_.EvaluateNP(dem,startValB,dx2,dy2,robotCenter+desc.chassisPosImage_,angleIdx,cx2,cy2);
        }

        results.caContactX1 = cx1;
        results.caContactY1 = cy1;

        results.caContactX2 = cx2;
        results.caContactY2 = cy2;
        results.caMinA = chassisTestA;
        results.caMinB = chassisTestB;

        if (chassisTestA < procConfig_.mapBaseHeight || chassisTestB < procConfig_.mapBaseHeight)
        {
            results.validState = PERS_CHASSISCOLLISION;
        }

    }


    return 0;

}


/*
int RobotModel::EvaluatePoseNP(const cv::Mat &dem, PoseEvalResults &results)
{


    const cv::Point2f pos(results.pose.x,results.pose.y);
    const float angle = results.pose.z;


    const WheelModel &wm0 = wheels_[0];
    const WheelModel &wm1 = wheels_[1];
    const WheelModel &wm2 = wheels_[2];
    const WheelModel &wm3 = wheels_[3];

    results.wheelAngles[0] = GetAngle(wm0,results.cmd,results.pose.z);
    results.wheelAngles[1] = GetAngle(wm1,results.cmd,results.pose.z);
    results.wheelAngles[2] = GetAngle(wm2,results.cmd,results.pose.z);
    results.wheelAngles[3] = GetAngle(wm3,results.cmd,results.pose.z);

    const int angleIdx = GetAngleIdx(angle);
    const int angleIdxW0 = GetWheelAngleIdx(results.wheelAngles[0],angleIdx,wm0);
    const int angleIdxW1 = GetWheelAngleIdx(results.wheelAngles[1],angleIdx,wm1);
    const int angleIdxW2 = GetWheelAngleIdx(results.wheelAngles[2],angleIdx,wm2);
    const int angleIdxW3 = GetWheelAngleIdx(results.wheelAngles[3],angleIdx,wm3);


    RobotDescriptor &desc = GetDescriptor(angleIdx);
    const cv::Point2f robotCenter = pos-desc.baseLinkPosImage_;

    int iz0,iz1,iz2,iz3;


    const int wr0 = wm0.EvaluateNP(dem,robotCenter+desc.wheelPositionsImage_[0],angleIdxW0, iz0,results.contactPoint0);
    const int wr1 = wm1.EvaluateNP(dem,robotCenter+desc.wheelPositionsImage_[1],angleIdxW1, iz1,results.contactPoint1);
    const int wr2 = wm2.EvaluateNP(dem,robotCenter+desc.wheelPositionsImage_[2],angleIdxW2, iz2,results.contactPoint2);
    const int wr3 = wm3.EvaluateNP(dem,robotCenter+desc.wheelPositionsImage_[3],angleIdxW3, iz3,results.contactPoint3);

    if (wr0 == -1 || wr1 == -1 || wr2 == -1 || wr3 == -1)
    {
        results.validState = PERS_OUTOFIMAGE;
        return -1;
    }

    const float z0 = -(float)(iz0-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;
    const float z1 = -(float)(iz1-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;
    const float z2 = -(float)(iz2-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;
    const float z3 = -(float)(iz3-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;


    const float d01 = z0-z1;
    const float d23 = z2-z3;

    const float conf0 = lw2_4*d01 + lw1_4*d23;


    float nAx,nAy,nBx,nBy;


    int cw1 = 0;
    int cw2 = 0;


    if (conf0 < 0)
    {// calc normal for wheel 0 & 2
        nAx = -w1_2*z3+w1mw2*z1+w1pw2*z0;
        nAy = -l_2*d01;
        nBx = w1mw2*z3+mw1mw2*z2+w2_2*z1;
        nBy = l_2*d23;

        cw1 = 0;
        cw2 = 2;




    }
    else
    { // calc normal for wheel 1 & 3
        nAx = -w1_2*z2+w1pw2*z1+w1mw2*z0;
        nAy = -l_2*d01;
        nBx = mw1mw2*z3+w1mw2*z2+w2_2*z0;
        nBy = l_2*d23;

        cw1 = 1;
        cw2 = 3;


    }
    const float lA = Q_rsqrt(nAx*nAx+nAy*nAy+lw1_4Sqr);
    const float lB = Q_rsqrt(nBx*nBx+nBy*nBy+lw2_4Sqr);


    const float n1x = nAx*lA;
    const float n1y = nAy*lA;

    results.n1.x = desc.cosa_*n1x - desc.sina_*n1y;
    results.n1.y = desc.sina_*n1x + desc.cosa_*n1y;
    results.n1.z = lw1_4*lA;

    const float n2x = nBx*lB;
    const float n2y = nBy*lB;

    results.n2.x = desc.cosa_*n2x - desc.sina_*n2y;
    results.n2.y = desc.sina_*n2x + desc.cosa_*n2y;
    results.n2.z = lw2_4*lB;

    const float agA = lw1_4*lA;
    const float agB = lw2_4*lB;

    results.tipAngle = results.n1.dot(results.n2);
    results.gravAngle = (agA<agB?agA:agB);

    results.a1 = agA;
    results.a2 = agB;

    results.zValues[0] = z0;
    results.zValues[1] = z1;
    results.zValues[2] = z2;
    results.zValues[3] = z3;

    if (chassisModel_.TestChassis())
    {

        const float dx1 = results.n1.z == 0 ? 0 : (-results.n1.x/fabs(results.n1.z))*procConfig_.heightPixelRatio;
        const float dy1 = results.n1.z == 0 ? 0 : (results.n1.y/fabs(-results.n1.z))*procConfig_.heightPixelRatio;

        const float dx2 = results.n2.z == 0 ? 0 : (-results.n2.x/fabs(results.n2.z))*procConfig_.heightPixelRatio;
        const float dy2 = results.n2.z == 0 ? 0 : (results.n2.y/fabs(-results.n2.z))*procConfig_.heightPixelRatio;

        const cv::Point2f w1  = desc.wheelPositionsImage_[cw1];//+wheels_[cw1].descriptors_[angleIdx].jointPosImg_;
        const cv::Point2f w2  = desc.wheelPositionsImage_[cw2];//+wheels_[cw2].descriptors_[angleIdx].jointPosImg_;


        const ChassisDescriptor chassisDesc = chassisModel_.GetDescriptorIdx(angleIdx);

        const cv::Point2f chassisPos = desc.chassisPosImage_ - chassisDesc.centerImg_;


        const cv::Point2f  wcPos1 = w1-(chassisPos);
        const cv::Point2f  wcPos2 = w2-(chassisPos);

        const float startValA = (results.zValues[cw1]*procConfig_.heightScale)- wcPos1.x*dx1 - wcPos1.y*dy1;
        const float startValB = (results.zValues[cw2]*procConfig_.heightScale)- wcPos2.x*dx2 - wcPos2.y*dy2;




        results.dx1 = dx1;
        results.dy1 = dy1;
        results.dx2 = dx2;
        results.dy2 = dy2;

        results.start1 = startValA;
        results.start2 = startValB;


        int cx1 = 0;
        int cy1 = 0;
        const int chassisTestA = chassisModel_.EvaluateNP(dem,startValA,dx1,dy1,robotCenter+desc.chassisPosImage_,angleIdx,cx1,cy1);

        int cx2 = 0;
        int cy2 = 0;
        const int chassisTestB = chassisModel_.EvaluateNP(dem,startValB,dx2,dy2,robotCenter+desc.chassisPosImage_,angleIdx,cx2,cy2);

        results.caMinA = chassisTestA;
        results.caMinB = chassisTestB;


    }

    results.validState = PERS_VALID;
    return 0;

}

int RobotModel::EvaluatePoseWS(const cv::Mat &dem, PoseEvalResults &results)
{
    const cv::Point2f pos(results.pose.x,results.pose.y);
    const float angle = results.pose.z;

    const WheelModel &wm0 = wheels_[0];
    const WheelModel &wm1 = wheels_[1];
    const WheelModel &wm2 = wheels_[2];
    const WheelModel &wm3 = wheels_[3];

    results.wheelAngles[0] = GetAngle(wm0,results.cmd,results.pose.z);
    results.wheelAngles[1] = GetAngle(wm1,results.cmd,results.pose.z);
    results.wheelAngles[2] = GetAngle(wm2,results.cmd,results.pose.z);
    results.wheelAngles[3] = GetAngle(wm3,results.cmd,results.pose.z);

    const int angleIdx = GetAngleIdx(angle);
    const int angleIdxW0 = GetWheelAngleIdx(results.wheelAngles[0],angleIdx,wm0);
    const int angleIdxW1 = GetWheelAngleIdx(results.wheelAngles[1],angleIdx,wm1);
    const int angleIdxW2 = GetWheelAngleIdx(results.wheelAngles[2],angleIdx,wm2);
    const int angleIdxW3 = GetWheelAngleIdx(results.wheelAngles[3],angleIdx,wm3);


    RobotDescriptor &desc = GetDescriptor(angleIdx);
    const cv::Point2f robotCenter = pos-desc.baseLinkPosImage_;

    int iz0,iz1,iz2,iz3;


    const int wr0 = wm0.EvaluateWS(dem,robotCenter+desc.wheelPositionsImage_[0],angleIdxW0, iz0,results.contactPoint0,results.wheelSupport[0]);
    const int wr1 = wm1.EvaluateWS(dem,robotCenter+desc.wheelPositionsImage_[1],angleIdxW1, iz1,results.contactPoint1,results.wheelSupport[1]);
    const int wr2 = wm2.EvaluateWS(dem,robotCenter+desc.wheelPositionsImage_[2],angleIdxW2, iz2,results.contactPoint2,results.wheelSupport[2]);
    const int wr3 = wm3.EvaluateWS(dem,robotCenter+desc.wheelPositionsImage_[3],angleIdxW3, iz3,results.contactPoint3,results.wheelSupport[3]);

    if (wr0 == -1 || wr1 == -1 || wr2 == -1 || wr3 == -1)
    {
        results.validState = PERS_OUTOFIMAGE;
        return -1;
    }

    const float z0 = -(float)(iz0-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;
    const float z1 = -(float)(iz1-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;
    const float z2 = -(float)(iz2-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;
    const float z3 = -(float)(iz3-procConfig_.imapBaseHeight)*procConfig_.heightScaleInv;


    const float d01 = z0-z1;
    const float d23 = z2-z3;

    const float conf0 = lw2_4*d01 + lw1_4*d23;


    float nAx,nAy,nBx,nBy;


    int cw1 = 0;
    int cw2 = 0;


    if (conf0 < 0)
    {// calc normal for wheel 0 & 2
        nAx = -w1_2*z3+w1mw2*z1+w1pw2*z0;
        nAy = -l_2*d01;
        nBx = w1mw2*z3+mw1mw2*z2+w2_2*z1;
        nBy = l_2*d23;

        cw1 = 0;
        cw2 = 2;




    }
    else
    { // calc normal for wheel 1 & 3
        nAx = -w1_2*z2+w1pw2*z1+w1mw2*z0;
        nAy = -l_2*d01;
        nBx = mw1mw2*z3+w1mw2*z2+w2_2*z0;
        nBy = l_2*d23;

        cw1 = 1;
        cw2 = 3;


    }
    const float lA = Q_rsqrt(nAx*nAx+nAy*nAy+lw1_4Sqr);
    const float lB = Q_rsqrt(nBx*nBx+nBy*nBy+lw2_4Sqr);


    const float n1x = nAx*lA;
    const float n1y = nAy*lA;

    results.n1.x = desc.cosa_*n1x - desc.sina_*n1y;
    results.n1.y = desc.sina_*n1x + desc.cosa_*n1y;
    results.n1.z = lw1_4*lA;

    const float n2x = nBx*lB;
    const float n2y = nBy*lB;

    results.n2.x = desc.cosa_*n2x - desc.sina_*n2y;
    results.n2.y = desc.sina_*n2x + desc.cosa_*n2y;
    results.n2.z = lw2_4*lB;

    const float agA = lw1_4*lA;
    const float agB = lw2_4*lB;

    results.tipAngle = results.n1.dot(results.n2);
    results.gravAngle = (agA<agB?agA:agB);

    results.a1 = agA;
    results.a2 = agB;

    results.zValues[0] = z0;
    results.zValues[1] = z1;
    results.zValues[2] = z2;
    results.zValues[3] = z3;

    if (chassisModel_.TestChassis())
    {

        const float dx1 = results.n1.z == 0 ? 0 : (-results.n1.x/fabs(results.n1.z))*procConfig_.heightPixelRatio;
        const float dy1 = results.n1.z == 0 ? 0 : (results.n1.y/fabs(-results.n1.z))*procConfig_.heightPixelRatio;

        const float dx2 = results.n2.z == 0 ? 0 : (-results.n2.x/fabs(results.n2.z))*procConfig_.heightPixelRatio;
        const float dy2 = results.n2.z == 0 ? 0 : (results.n2.y/fabs(-results.n2.z))*procConfig_.heightPixelRatio;

        const cv::Point2f w1  = desc.wheelPositionsImage_[cw1];//+wheels_[cw1].descriptors_[angleIdx].jointPosImg_;
        const cv::Point2f w2  = desc.wheelPositionsImage_[cw2];//+wheels_[cw2].descriptors_[angleIdx].jointPosImg_;


        const ChassisDescriptor chassisDesc = chassisModel_.GetDescriptorIdx(angleIdx);

        const cv::Point2f chassisPos = desc.chassisPosImage_ - chassisDesc.centerImg_;


        const cv::Point2f  wcPos1 = w1-(chassisPos);
        const cv::Point2f  wcPos2 = w2-(chassisPos);

        const float startValA = (results.zValues[cw1]*procConfig_.heightScale)- wcPos1.x*dx1 - wcPos1.y*dy1;
        const float startValB = (results.zValues[cw2]*procConfig_.heightScale)- wcPos2.x*dx2 - wcPos2.y*dy2;




        results.dx1 = dx1;
        results.dy1 = dy1;
        results.dx2 = dx2;
        results.dy2 = dy2;

        results.start1 = startValA;
        results.start2 = startValB;


        int cx1 = 0;
        int cy1 = 0;
        const int chassisTestA = chassisModel_.EvaluateNP(dem,startValA,dx1,dy1,robotCenter+desc.chassisPosImage_,angleIdx,cx1,cy1);

        int cx2 = 0;
        int cy2 = 0;
        const int chassisTestB = chassisModel_.EvaluateNP(dem,startValB,dx2,dy2,robotCenter+desc.chassisPosImage_,angleIdx,cx2,cy2);

        results.caMinA = chassisTestA;
        results.caMinB = chassisTestB;


    }

    results.validState = PERS_VALID;
    return 0;

}
*/

void RobotModel::SetupRobot(ModelBasedPlannerConfig &config)
{
    std::vector<WheelConfig> wheelConfs = config.GetWheelConfigs();

    config.procConfig_.Setup();
    SetupRobot(config.procConfig_,config.robotConfig_,wheelConfs,config.chassisConfig_);





}



void RobotModel::SetupRobot(const ProcConfig &procConfig, const RobotConfig &robotConfig,  const std::vector<WheelConfig> &wheelConfigs, const ChassisConfig &chassisConfig)
{

    procConfig_ = procConfig;
    config_ = robotConfig;


    chassisModel_.SetupChassis(procConfig,chassisConfig);

    //config_.testChassis_ = chassisConfig.testChassis;


    wheels_.resize(wheelConfigs.size());

    for (unsigned int i = 0; i < wheels_.size();++i)
    {
        wheels_[i].SetupWheel(procConfig_,wheelConfigs[i]);
    }

    std::vector<int> hasCopy;
    hasCopy.resize(wheels_.size(),0);
    for (unsigned int i = 0; i < wheels_.size();++i)
    {
        if (hasCopy[i] == 1) continue;
        for (unsigned int j = i+1; j < wheels_.size();++j)
        {
            if (hasCopy[j] == 1) continue;
            WheelModel *w1 = &wheels_[i];
            WheelModel *w2 = &wheels_[j];

            if (w1->latRadiusImg_ == w2->latRadiusImg_ && w1->radiusImg_ == w2->radiusImg_ && w1->widthImg_ == w2->widthImg_)
            {
                for (unsigned int tl = 0; tl < w1->descriptors_.size(); ++tl)
                {
                    w2->descriptors_[tl].image_ = w1->descriptors_[tl].image_;
                    hasCopy[j] = 1;
                }
            }


        }
    }



    cv::Point2f wpos0 = wheels_[0].GetWheelPos();
    cv::Point2f wpos1 = wheels_[1].GetWheelPos();
    cv::Point2f wpos2 = wheels_[2].GetWheelPos();
    cv::Point2f wpos3 = wheels_[3].GetWheelPos();


    frontWidthHalf_ = (wpos3.y-wpos2.y)/2.0f;
    backWidthHalf_ = (wpos0.y-wpos1.y)/2.0f;

    robotLengthHalf_ = (wpos3.x-wpos0.x)/2.0f;

    //float th1 = (wpos3.x-wpos0.x)/2.0f;
    //float th2 = (wpos2.x-wpos1.x)/2.0f;

    //robotCenter_.x = (wpos3.x+wpos2.x+wpos1.x+wpos0.x)/4.0f;
    //robotCenter_.y = (wpos3.y+wpos2.y+wpos1.y+wpos0.y)/4.0f;

    float w1 = backWidthHalf_;
    float w2 = frontWidthHalf_;
    float l = robotLengthHalf_;

    lw1_4 = l*w1*4.0;
    lw2_4 = l*w2*4.0;
    l_2 = l*2.0;
    w1_2 = w1*2.0;
    w2_2 = w2*2.0;
    w1pw2 = w1+w2;
    w1mw2 = w1-w2;
    mw1mw2 = -w1-w2;

    lw1_4Sqr = lw1_4*lw1_4;
    lw2_4Sqr = lw2_4*lw2_4;




    float curAngle = 0;
    //angleStep_ = 2.0*CV_PI/(double)numAngleSteps;

    for (int i = 0; i < procConfig.numAngleStep;i++)
    {
        curAngle = procConfig.angleStep*(float)i;
        cv::Mat rotMat = cv::getRotationMatrix2D(cv::Point2f(0,0),curAngle*(180.0/CV_PI),1.0);

        RobotDescriptor desc;

        desc.rotMat_ = rotMat;

        desc.sina_ = sin(curAngle);
        desc.cosa_ = cos(curAngle);

        cv::Mat twp(3,1,CV_64F);
        twp.at<double>(0,0) = config_.baseLinkPosCoord.x;
        twp.at<double>(1,0) = config_.baseLinkPosCoord.y;
        twp.at<double>(2,0) = 1.0;
        cv::Mat res2 = rotMat*(twp);


        desc.baseLinkPosImage_ = cv::Point2f(res2.at<double>(0,0)*procConfig.pixelSizeInv, -res2.at<double>(1,0)*procConfig.pixelSizeInv);


        twp.at<double>(0,0) = chassisConfig.chassisPosRobot.x;
        twp.at<double>(1,0) = chassisConfig.chassisPosRobot.y;
        twp.at<double>(2,0) = 1.0;
        cv::Mat res3 = rotMat*(twp);

        desc.chassisPosImage_ = cv::Point2f(res3.at<double>(0,0)*procConfig.pixelSizeInv, -res3.at<double>(1,0)*procConfig.pixelSizeInv);



        for (unsigned int i = 0; i < wheels_.size();i++)
        {
            //cv::Vec3f twp(wheels_[i].wheelPosCoord_.x,wheels_[i].wheelPosCoord_.y,1);
            //cv::Mat twp(3,1,CV_64F);
            twp.at<double>(0,0) = wheels_[i].config_.wheelPosRobot.x;
            twp.at<double>(1,0) = wheels_[i].config_.wheelPosRobot.y;
            twp.at<double>(2,0) = 1.0;

            cv::Mat res = rotMat*(twp);
            //desc.wheelPositionsCoord_.push_back(cv::Point2f(res.at<double>(0,0), res.at<double>(1,0)));
            desc.wheelPositionsImage_.push_back(cv::Point2f(res.at<double>(0,0)*procConfig.pixelSizeInv, -res.at<double>(1,0)*procConfig.pixelSizeInv ));

        }


        //desc.gravCenterImage_ = intersection(desc.wheelPositionsImage_[0],desc.wheelPositionsImage_[2],desc.wheelPositionsImage_[1],desc.wheelPositionsImage_[3]);

        /*
        if (testChassis_)
        {
            twp.at<double>(0,0) = chassisModel_.GetWheelPos().x;
            twp.at<double>(1,0) = chassisModel_.GetWheelPos().y;
            twp.at<double>(2,0) = 1.0;

            cv::Mat res = rotMat*(twp);
           desc.chassisPosCoord_ = (cv::Point2f(res.at<double>(0,0), res.at<double>(1,0)));
           desc.chassisPosImage_ = (cv::Point2f(res.at<double>(0,0)*GConfig::pC.pixelSizeInv_, -res.at<double>(1,0)*GConfig::pC.pixelSizeInv_ ));

        }
        */


        descriptors_.push_back(desc);


    }

}
