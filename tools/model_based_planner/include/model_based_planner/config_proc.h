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

        imagePosBLMinX = 0;
        imagePosBLMinY = 0;

        validThresholdFactor = 0.95;
        wheelSupportThresholdFactor = 1.2;
        convertImage = false;

        Setup();


    }

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


    int numAngleStep;

    float heightScale;
    short mapBaseHeight;
    short notVisibleLevel;

    short wheelGroundLevel;
    short maxHeight;
    float pixelSize;



    float imagePosBLMinX,imagePosBLMinY;

    float validThresholdFactor;
    float wheelSupportThresholdFactor;

    bool convertImage;


//calculated
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
