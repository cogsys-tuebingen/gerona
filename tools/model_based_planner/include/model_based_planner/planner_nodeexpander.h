#ifndef PLANNER_NODEEXPANDER_H
#define PLANNER_NODEEXPANDER_H



#include "plannerutils.h"

#include <config_planner.h>

/**
 * @brief Base class for different node expanders
 */
struct NodeExpander_Base
{
    void SetConfig(const PlannerExpanderConfig &config, const float pixelSize)
    {
        config_ = config;
        numSplitsPerSide_ = (float)(config.numSplits/2);

        numSplitsPerSideFirst_ = (float)(config.firstLevelSplits/2);

        numSplitsLinearFirst_ = (float) (config.firstLevelLinearSplits/2);

        deltaLinImage_ = config.firstLevelDeltaLinear/pixelSize;

        minVelImage_ = config.minLinVel/pixelSize;
        maxVelImage_ = config.maxLinVel/pixelSize;


    }



    inline void GetParamsAng(int &lvl, int &numSplits, float &deltaT, float &splitsPerSide)
    {
        numSplits = config_.numSplits;
        deltaT = config_.deltaTheta;
        splitsPerSide = numSplitsPerSide_;

        if (config_.firstLevelSplits > 0 && lvl < 1)
        {
            numSplits = config_.firstLevelSplits;
            deltaT = config_.firstLevelDeltaTheta;
            splitsPerSide = numSplitsPerSideFirst_;


        }
    }

    PlannerExpanderConfig config_;
    float numSplitsPerSide_;
    float numSplitsPerSideFirst_;
    float numSplitsLinearFirst_;
    float deltaLinImage_;
    float minVelImage_,maxVelImage_;


};

/**
 * @brief Node expander with linear and angular velocity changes
 */
struct NodeExpander_LAVT_T : public NodeExpander_Base
{
    NodeExpander_LAVT_T()
    {

    }


    inline int Expand(int lvl, const cv::Point2f &curCmd, std::vector<cv::Point2f> &resCmds)
    {
        int resultSplits = 0;


        int numSplits = config_.numSplits;
        float deltaT = config_.deltaTheta;
        float splitsPerSide = numSplitsPerSide_;
        int numLinSplits = 1;
        float numLinSplitsPerSide = 0;
        float deltaLin = 0;

        if (config_.firstLevelSplits > 0 && lvl < 1)
        {
            numSplits = config_.firstLevelSplits;
            deltaT = config_.firstLevelDeltaTheta;
            splitsPerSide = numSplitsPerSideFirst_;

        }

        if (config_.firstLevelLinearSplits > 0 && lvl < 1)
        {
            numLinSplits = config_.firstLevelLinearSplits;
            deltaLin = deltaLinImage_;
            numLinSplitsPerSide = numSplitsLinearFirst_;
        }

        cv::Point2f cmd = curCmd;


        cmd.y = -deltaT*splitsPerSide;
        cmd.x += -deltaLin*numLinSplitsPerSide;

        float orgDeltaT = cmd.y;

        bool capLinMin = false;
        bool capLinMax = false;
        //bool doExpand = true;


        if (cmd.x >= maxVelImage_)
        {
            cmd.x = maxVelImage_;
            numLinSplits = 1;
        }

        if (cmd.x+ (float)(numLinSplits-1)*deltaLin < minVelImage_)
        {
            cmd.x = minVelImage_;
            numLinSplits = 1;
        }


        //cmd.y = -deltaTheta_*(float)numSplitsPerSide_;
        for (int ls = 0; ls < numLinSplits;++ls)
        {
            //doExpand = true;
            capLinMin = false;
            capLinMax = false;

            if (cmd.x+deltaLinImage_ <= minVelImage_-COMMANDEPSILON)
            {
                continue;
            }

            if (cmd.x-deltaLinImage_ >= maxVelImage_ + COMMANDEPSILON )
            {
                continue;
            }


            if (cmd.x <= minVelImage_-COMMANDEPSILON && cmd.x+deltaLinImage_ >= minVelImage_-COMMANDEPSILON)
            {
                capLinMin = true;
            }

            if (cmd.x >= maxVelImage_-COMMANDEPSILON && cmd.x-deltaLinImage_ <= maxVelImage_-COMMANDEPSILON)
            {
                capLinMax = true;
            }

            //if (cmd.x >= minVelImage_-COMMANDEPSILON )
            {
                for (int tl = 0; tl < numSplits;++tl)
                {
                    if (std::abs(cmd.y) < COMMANDEPSILON) cmd.y = 0;
                    resCmds[resultSplits] = cmd;
                    if (capLinMin) resCmds[resultSplits].x = minVelImage_;
                    if (capLinMax) resCmds[resultSplits].x = maxVelImage_;

                    cmd.y += deltaT;
                    ++resultSplits;
                }
            }
            cmd.x += deltaLin;
            cmd.y = orgDeltaT;
        }

        return resultSplits;
    }

    inline int ExpandOld(int lvl, const cv::Point2f &curCmd, std::vector<cv::Point2f> &resCmds)
    {
        int resultSplits = 0;


        int numSplits = config_.numSplits;
        float deltaT = config_.deltaTheta;
        float splitsPerSide = numSplitsPerSide_;
        int numLinSplits = 1;
        float numLinSplitsPerSide = 0;
        float deltaLin = 0;

        if (config_.firstLevelSplits > 0 && lvl < 1)
        {
            numSplits = config_.firstLevelSplits;
            deltaT = config_.firstLevelDeltaTheta;
            splitsPerSide = numSplitsPerSideFirst_;

        }

        if (config_.firstLevelLinearSplits > 0 && lvl < 1)
        {
            numLinSplits = config_.firstLevelLinearSplits;
            deltaLin = deltaLinImage_;
            numLinSplitsPerSide = numSplitsLinearFirst_;
        }

        cv::Point2f cmd = curCmd;


        cmd.y = -deltaT*splitsPerSide;
        cmd.x += -deltaLin*numLinSplitsPerSide;

        float orgDeltaT = cmd.y;

        bool capLinMin = false;
        bool capLinMax = false;
        //bool doExpand = true;

        //cmd.y = -deltaTheta_*(float)numSplitsPerSide_;
        for (int ls = 0; ls < config_.firstLevelLinearSplits;++ls)
        {
            //doExpand = true;
            capLinMin = false;
            capLinMax = false;

            if (cmd.x+deltaLinImage_ <= minVelImage_-COMMANDEPSILON || cmd.x-deltaLinImage_ >= maxVelImage_ + COMMANDEPSILON )
            {
                if (ls == 0 && cmd.x > maxVelImage_)
                {
                    ls = config_.firstLevelLinearSplits;
                    capLinMax = true;
                }
                else if (ls == config_.firstLevelLinearSplits-1 && cmd.x < minVelImage_)
                {
                    capLinMin = true;
                } else continue;
            }
            if (cmd.x <= minVelImage_-COMMANDEPSILON && cmd.x+deltaLinImage_ >= minVelImage_-COMMANDEPSILON)
            {
                capLinMin = true;
            }

            if (cmd.x >= maxVelImage_-COMMANDEPSILON && cmd.x-deltaLinImage_ <= maxVelImage_-COMMANDEPSILON)
            {
                capLinMax = true;
            }

            //if (cmd.x >= minVelImage_-COMMANDEPSILON )
            {
                for (int tl = 0; tl < numSplits;++tl)
                {
                    if (std::abs(cmd.y) < COMMANDEPSILON) cmd.y = 0;
                    resCmds[resultSplits] = cmd;
                    if (capLinMin) resCmds[resultSplits].x = minVelImage_;
                    if (capLinMax) resCmds[resultSplits].x = maxVelImage_;

                    cmd.y += deltaT;
                    ++resultSplits;
                }
            }
            cmd.x += deltaLin;
            cmd.y = orgDeltaT;
        }

        return resultSplits;
    }

    /*
        else
        {
            int numSplits;
            float deltaT;
            float splitsPerSide;

            GetParams(lvl,numSplits,deltaT,splitsPerSide);

            cv::Point2f cmd = curCmd;

            cmd.y += -deltaT*splitsPerSide;
            //cmd.y = -deltaTheta_*(float)numSplitsPerSide_;

            for (int tl = 0; tl < numSplits;++tl)
            {
                if (std::abs(cmd.y) < COMMANDEPSILON) cmd.y = 0;
                resCmds[tl] = cmd;

                cmd.y += deltaT;

            }
        }


    }
*/

};



/**
 * @brief Node expander with constant linear velocity, only angular velocity changes. The angular velocity is added up at each tree level.
 * The resulting angular velocity therefore can be deltaT*maxLevels
 */
struct NodeExpander_AVT_T : public NodeExpander_Base
{
    NodeExpander_AVT_T()
    {

    }


    inline int Expand(int lvl, const cv::Point2f &curCmd, std::vector<cv::Point2f> &resCmds)
    {
        int numSplits;
        float deltaT;
        float splitsPerSide;

        GetParamsAng(lvl,numSplits,deltaT,splitsPerSide);

        cv::Point2f cmd = curCmd;

        cmd.y += -deltaT*(float)splitsPerSide;
        //cmd.y = -deltaTheta_*(float)numSplitsPerSide_;

        for (int tl = 0; tl < numSplits;++tl)
        {
            if (std::abs(cmd.y) < COMMANDEPSILON) cmd.y = 0;
            resCmds[tl] = cmd;

            cmd.y += deltaT;

        }

        return numSplits;

    }


};


/**
 * @brief Node expander with constant linear velocity, only angular velocity changes. The maximum angular velocity is deltaT.
 */
struct NodeExpander_AVNI_T : public NodeExpander_Base
{
    NodeExpander_AVNI_T()
    {

    }


    inline int Expand(int lvl, const cv::Point2f &curCmd, std::vector<cv::Point2f> &resCmds)
    {
        int numSplits;
        float deltaT;
        float splitsPerSide;

        GetParamsAng(lvl,numSplits,deltaT,splitsPerSide);

        cv::Point2f cmd = curCmd;

        cmd.y = -deltaT*(float)splitsPerSide;

        for (int tl = 0; tl < numSplits;++tl)
        {
            if (std::abs(cmd.y) < COMMANDEPSILON) cmd.y = 0;
            resCmds[tl] = cmd;

            cmd.y += deltaT;

        }

        return numSplits;

    }


};


#endif // PLANNER_NODEEXPANDER_H

