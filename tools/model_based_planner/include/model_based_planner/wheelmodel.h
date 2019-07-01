#ifndef WHEELMODEL_H
#define WHEELMODEL_H

#include "wheeldescriptor.h"
#include "config_robot.h"
#include "config_proc.h"
#include "poseevalresults.h"


#define WM_USE_WHEEL_SUPPORT 1

/**
 * @brief Wheel model implementation
 */
class WheelModel
{
public:
    WheelModel();

    /**
     * @brief Creates a new wheel from configs
     */
    void SetupWheel(const ProcConfig& procConfig, const WheelConfig &conf);


    /**
     * @brief Get ith wheel descriptor
     */
    WheelDescriptor& GetDescriptorIdx(int i) {return descriptors_[i];}

    /**
     * @brief Get the position of the wheel joint in image coordinates
     */
    inline const cv::Point2i GetImagePos(const cv::Point2f &pos, const WheelDescriptor& desc) const
    {
        return cv::Point2i(round(pos.x- desc.jointPosImg_.x),round(pos.y- desc.jointPosImg_.y));

    }

    /**
     * @brief Evaluate wheel on the current DEM
     */
    int Evaluate(const cv::Mat &dem,const cv::Point2f &pos, WheelEvalResults &results) const;



    /**
     * @brief Returns true if the wheel can turn or is a caster wheel
     */
    inline bool IsTurnable() const
    {
        return config_.isTurnableWheel;
    }

    /**
     * @brief Get wheel posiition on robot
     */
    cv::Point2f GetWheelPos() {return config_.wheelPosRobot;}


    WheelConfig config_;
    std::vector<WheelDescriptor> descriptors_;
    cv::Point2f jointPosImg_;
    float radiusImg_,latRadiusImg_, widthImg_;
    float angleStep_;
    int wheelSupportThreshold_;


};

#endif // WHEELMODEL_H
