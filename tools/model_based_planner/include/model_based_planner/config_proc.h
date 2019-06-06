#ifndef PROCCONFIG_H
#define PROCCONFIG_H
#include <opencv2/core/core.hpp>

/**
 * @brief Config describing the parameters of the DEM required for creating the vehicle model
 *
 */
struct ProcConfig
{
public:
    ProcConfig()
    {
        numAngleStep = 360;
        heightScale = 1000.0;
        mapBaseHeight = 10000;
        notVisibleLevel = 1000;

        wheelGroundLevel = 20000;
        maxHeight = 30000;
        pixelSize  = 0.01;


        validThresholdFactor = 0.95;
        wheelSupportThresholdFactor = 1.2;

        Setup();


    }


    /**
     * @brief calculate values which are not set manually
     */
    void Setup(){
        angleStep = (CV_PI*2.0)/(double)numAngleStep;
        angleStepInv = 1.0/angleStep;
        pixelSizeInv = 1.0/pixelSize;
        heightScaleInv = 1.0/heightScale;
        heightPixelRatio = heightScale/pixelSizeInv;

        validThreshold = -(mapBaseHeight*heightScaleInv)*validThresholdFactor;
        notVisibleThreshold = -((mapBaseHeight-notVisibleLevel)*heightScaleInv)*validThresholdFactor;
        imapBaseHeight = mapBaseHeight;
    }


    /**
     * @brief number of angle steps for testing vehicle orientation. 360 is default and usually a good choice.
     */
    int numAngleStep;

    /**
     * @brief height scale for pixel values. For 1000 a gray value increase equals 1mm
     */
    float heightScale;
    /**
     * @brief the map base height describes where zero height is located. defaults to 10000, with a height scale of 1000 this gives 10m.
     */
    short mapBaseHeight;
    /**
     * @brief  a pixel value below this value is considered as not seen by sensor.
     */
    short notVisibleLevel;

    /**
     * @brief for subtraction the wheels are modelled at this height. defaults to 20000, i.e. 20m for height scale 1000.
     */
    short wheelGroundLevel;
    /**
     * @brief max height, used for displaying and rendering
     */
    short maxHeight;
    /**
     * @brief the size of one pixel in meters. Values ~ 0.01m are recommended
     */
    float pixelSize;

    /**
     * @brief A wheel z position is treated as invalid if it is below (1-validThresholdFactor)*mapBaseHeight
     */
    float validThresholdFactor;
    /**
     * @brief if the distance of a wheel pixel to the ground is above wheelSupportThresholdFactor*wheelRadius it is considered invalid
     */
    float wheelSupportThresholdFactor;


// pre calculated
    float angleStep;
    float angleStepInv;

    float pixelSizeInv;
    float heightScaleInv;
    float heightPixelRatio;
    float validThreshold;
    float notVisibleThreshold;
    int imapBaseHeight;





};

#endif // PROCCONFIG_H
