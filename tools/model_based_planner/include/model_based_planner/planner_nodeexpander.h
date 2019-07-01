#ifndef PLANNER_NODEEXPANDER_H
#define PLANNER_NODEEXPANDER_H



#include "plannerutils.h"

#include <config_planner.h>

/**
 * @brief Interface class for different node expanders. Node expanders create a new set of command velocities for a given input command velocity.
 */

struct INodeExpander
{
    typedef std::shared_ptr<INodeExpander > Ptr;

    virtual int Expand(int lvl, const cv::Point2f &curCmd, std::vector<cv::Point2f> &resCmds) = 0;
    virtual void SetConfig(const PlannerExpanderConfig &config, const float pixelSize) = 0;


};

/**
 * @brief Base class for node expandes
 */
struct NodeExpander_Base : INodeExpander
{
    NodeExpander_Base()
    {

    }


    /**
     * @brief Get all required parameters from configuration
     */
    void SetConfig(const PlannerExpanderConfig &config, const float pixelSize)
    {
        config_ = config;
        numSplitsPerSide_ = (float)(config.numSplits/2);

        numSplitsPerSideFirst_ = (float)(config.firstLevelSplits/2);

        numSplitsLinearPerSideFirst_ = (float) (config.firstLevelLinearSplits/2);

        deltaLinImage_ = config.firstLevelDeltaLinear/pixelSize;

        minLinVelImage_ = config.minLinVel/pixelSize;
        maxLinVelImage_ = config.maxLinVel/pixelSize;
        maxAngVelImage_ = config.maxAngVel;


    }



    /**
     * @brief Get angular velocity parameters
     */
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

protected:
    PlannerExpanderConfig config_;
    float numSplitsPerSide_;
    float numSplitsPerSideFirst_;
    float numSplitsLinearPerSideFirst_;
    float deltaLinImage_;
    float minLinVelImage_,maxLinVelImage_;
    float maxAngVelImage_;


};

/**
 * @brief Node expander with linear and angular velocity changes
 */
struct NodeExpander_LAVT : public NodeExpander_Base
{
    typedef std::shared_ptr<NodeExpander_LAVT> Ptr;
    static NodeExpander_LAVT::Ptr Create(){ return std::make_shared< NodeExpander_LAVT >() ; }

    static constexpr const char* const NE_NAME = "linear_angular_vel_rel";


    NodeExpander_LAVT()
    {

    }

    #define LINVELEPSILON 0.01f

    /**
     * @brief Get new linear velocities, clamp if above or below thresholds
     */
    void GetVels(const float curVel,const float minvel,const float maxvel, const float delta, const int stepsPerSide, std::vector<float> &linVels) const
    {

        linVels.clear();

        float clampedvel = curVel;

        if (clampedvel <= minvel+LINVELEPSILON)
        {
            clampedvel = minvel;

        }
        else
        {
            for (int tl = 1; tl < stepsPerSide+1;tl++)
            {
                float tlVel = clampedvel - delta * (float)tl;

                if (tlVel > minvel+LINVELEPSILON)
                {
                    linVels.push_back(tlVel);
                }
                else
                {
                    linVels.push_back(minvel);
                    break;
                }


            }
        }
        if (clampedvel >= maxvel-LINVELEPSILON)
        {
            clampedvel = maxvel;

        }
        else
        {
            for (int tl = 1; tl < stepsPerSide+1;tl++)
            {
                float tlVel = clampedvel + delta * (float)tl;

                if (tlVel < maxvel-LINVELEPSILON)
                {
                    linVels.push_back(tlVel);
                }
                else
                {
                    linVels.push_back(maxvel);
                    break;
                }
            }
        }

        linVels.push_back(clampedvel);

        std::sort(linVels.begin(),linVels.end());

    }


    /**
     * @brief Create set of command velocities
     */
    inline int Expand(int lvl, const cv::Point2f &curCmd, std::vector<cv::Point2f> &resCmds)
    {
        int resSplites = 0;

        float deltaT = config_.deltaTheta;
        float splitsPerSide = numSplitsPerSide_;
        float numLinSplitsPerSide = 0;
        float deltaLin = 0;

        if (config_.firstLevelSplits > 0 && lvl < 1)
        {
            deltaT = config_.firstLevelDeltaTheta;
            splitsPerSide = numSplitsPerSideFirst_;

        }

        if (config_.firstLevelLinearSplits > 0 && lvl < 1)
        {
            deltaLin = deltaLinImage_;
            numLinSplitsPerSide = numSplitsLinearPerSideFirst_;
        }


        std::vector<float> linVels;
        if (numLinSplitsPerSide == 0) linVels.push_back(curCmd.x);
        else GetVels(curCmd.x,minLinVelImage_,maxLinVelImage_,deltaLin,numLinSplitsPerSide,linVels);


        std::vector<float> angVels;
        if (splitsPerSide == 0) linVels.push_back(curCmd.y);
        else GetVels(curCmd.y,-maxAngVelImage_,maxAngVelImage_,deltaT,splitsPerSide,angVels);

        for(auto const& linVel: linVels) {
            for(auto const& angVel: angVels) {
                cv::Point2f cmd(linVel,angVel);
                resCmds[resSplites] = cmd;

                resSplites++;
            }
        }

        return resSplites;
    }

};



/**
 * @brief Node expander with constant linear velocity, only angular velocity changes. The angular velocity is added up at each tree level.
 * The resulting angular velocity therefore can be deltaT*maxLevels
 */
struct NodeExpander_AVT : public NodeExpander_Base
{
    typedef std::shared_ptr<NodeExpander_AVT> Ptr;
    static NodeExpander_AVT::Ptr Create(){ return std::make_shared< NodeExpander_AVT >() ; }

    static constexpr const char* const NE_NAME = "angular_vel_rel";

    NodeExpander_AVT()
    {

    }


    /**
     * @brief Create set of command velocities
     */
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
 * @brief Node expander with constant linear velocity, only angular velocity changes. The maximum angular velocity is deltaT*splitsPerSide.
 */
struct NodeExpander_AVNI : public NodeExpander_Base
{
    typedef std::shared_ptr<NodeExpander_AVNI> Ptr;
    static NodeExpander_AVNI::Ptr Create(){ return std::make_shared< NodeExpander_AVNI >() ; }

    static constexpr const char* const NE_NAME = "angular_vel";

    NodeExpander_AVNI()
    {

    }


    /**
     * @brief Create set of command velocities
     */
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



/**
 * @brief Node expander with constant linear velocity, only angular velocity changes. Reduces linear velocity for larger angular velocities.
 */
struct NodeExpander_AVLR : public NodeExpander_Base
{
    typedef std::shared_ptr<NodeExpander_AVLR> Ptr;
    static NodeExpander_AVLR::Ptr Create(){ return std::make_shared< NodeExpander_AVLR >() ; }

    static constexpr const char* const NE_NAME = "angular_vel_lin_red";

    NodeExpander_AVLR()
    {

    }


    /**
     * @brief Create set of command velocities
     */
    inline int Expand(int lvl, const cv::Point2f &curCmd, std::vector<cv::Point2f> &resCmds)
    {
        int numSplits;
        float deltaT;
        float splitsPerSide;

        GetParamsAng(lvl,numSplits,deltaT,splitsPerSide);

        cv::Point2f cmd = curCmd;

        cmd.y = -deltaT*(float)splitsPerSide;
        //cmd.y += -deltaT*(float)splitsPerSide;

        const float maxAngVel  = std::abs(deltaT*(float)splitsPerSide);
        const float avRange =   maxAngVel-maxAngVelImage_;

        for (int tl = 0; tl < numSplits;++tl)
        {
            if (std::abs(cmd.y) < COMMANDEPSILON) cmd.y = 0;

            const float aAbs = std::abs(cmd.y);

            if (aAbs > maxAngVelImage_ && lvl <= 1)
            {
                const float diff = aAbs - maxAngVelImage_;
                const float diffLinVel = maxLinVelImage_ - minLinVelImage_;
                const float linVel = minLinVelImage_ + diffLinVel * (avRange-diff)/avRange;
                cmd.x = linVel;
            }
            else cmd.x = curCmd.x;


            resCmds[tl] = cmd;

            cmd.y += deltaT;



        }

        return numSplits;

    }


};







#endif // PLANNER_NODEEXPANDER_H

