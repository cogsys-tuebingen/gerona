#ifndef CHASSISMODEL_H
#define CHASSISMODEL_H


#include "chassisdescriptor.h"
#include "config_robot.h"
#include "config_proc.h"
#include "wheeldescriptor.h"



/**
 * @brief ChassisModel model implementation
 */
class ChassisModel
{
public:
    ChassisModel();

    /**
     * @brief Setup chassis model
     */
    void SetupChassis(const ProcConfig& procConfig, const ChassisConfig &conf);

    /**
     * @brief Get descriptor at index i without checking
     */
    const ChassisDescriptor& GetDescriptorIdx(int i) const {return descriptors_[i];}


    /**
     * @brief Test for chassis collision at given pos and angle and determines contact points, requires startVal, dx, dy from the pose estimate
     */
    int Evaluate(const cv::Mat &dem,const float &startVal, const float &dx, const float &dy, const cv::Point2f &pos, const int &angleIdx, int &cx, int &cy) const;
    /**
     * @brief Test for chassis collision at given pos and angle without contact points(faster), requires startVal, dx, dy from the pose estimate
     */
    int EvaluateNP(const cv::Mat &dem,const float &startVal, const float &dx, const float &dy, const cv::Point2f &pos, const int &angleIdx, int &cx, int &cy) const;

    /**
     * @brief True if chassis testing is set
     */
    inline bool TestChassis() const {return config_.testChassis;}



private:
    ChassisConfig config_;
    /**
     * @brief vector conaining all chassis descriptors for each orientation
     */
    std::vector<ChassisDescriptor> descriptors_;


};

#endif // CHASSISMODEL_H
