#ifndef RGBD_ZIMAGE
#define RGBD_ZIMAGE

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils_depth_image.h"


/// Transposing the image can speed up processing, but also coordinates have to be adjusted accordingly
//#define TRANSPOSE_TRANSFORM 1

/**
 * @brief Helper class for performing the orthogonal projection and transform the points into a elevation image
 */

class ZImageProc
{
public:

    float cx,cy,fx,fy,t1,t2,t3,r11,r12,r13,r21,r22,r23,r31,r32,r33;
    float fxi,fyi;

    float pixelResolution_;
    float minXVal_;
    float minYVal_;
    float minAssignValue_;
    float minDepthThreshold_;


    ZImageProc()
    {
        minDepthThreshold_ = 0.4f;
        minAssignValue_ = 0.5;


    }


    /**
     * @brief Set camera parameters
     */
    void SetupCam( float tfx, float tfy, float tcx, float tcy)
    {

        fx = tfx;
        fxi = 1.0f/fx;
        fy = tfy;
        fyi = 1.0f/fy;

        cx = tcx;
        cy = tcy;


    }



    /**
     * @brief Uses an inverse bilinear interpolation to interpolate the z values of neighbouring pixels (without min max calculation)
     */
    void ProcessDepthImage(const cv::Mat &inD, cv::Mat &zImg, cv::Mat &assignVal, const float scale, const float baseLevel, const float zeroLevel, const float minVisX = 0.0f)
    {

        int resolutionX = zImg.cols;
        int resolutionY = zImg.rows;

        float resolutionXf_ = (float) (resolutionX-1);
        float resolutionYf_ = (float) (resolutionY-1);



        UtilsDepthImage::SetToZero(zImg);//.setTo(0);
        UtilsDepthImage::SetToZero(assignVal);//.setTo(0);

        float *zImgRes = zImg.ptr<float>();
        float *assignRes = assignVal.ptr<float>();

        float posX,posY,posZ;
        float flposX,flposY;

        float wtl,wbl,wtr,wbr;

        float wl,wr,wb,wt;
        int idxFTL,idxFTLY,stepF = assignVal.cols;


        int xl,yl;

        const float *ptrD;

        float tx,ty,tz;

        float rx,ry,rz;

        float oz;

        for  (yl = 0; yl < inD.rows;++yl)
        {
            ptrD = inD.ptr<float>(yl);

            for  (xl = 0; xl < inD.cols;++xl)
            {
                oz = ptrD[xl];
                tx = ((float(xl)-cx)*fxi)*oz;
                ty = ((float(yl)-cy)*fyi)*oz;
                tz = oz;

                rx = tx*r11+ty*r12+tz*r13+t1;
                ry = tx*r21+ty*r22+tz*r23+t2;
                rz = tx*r31+ty*r32+tz*r33+t3;

                if (std::isnan(tx)) continue;


#ifndef TRANSPOSE_TRANSFORM

                // normal

                posX = (rx-minXVal_) * pixelResolution_;
                posY = (ry-minYVal_) * pixelResolution_;
                posZ = rz;

#else

                // Transposed
                posY = (rx-minXVal_) * pixelResolution_;
                posX = (ry-minYVal_) * pixelResolution_;
                posZ = rz;
#endif

                flposX = floor(posX);
                flposY = floor(posY);

                if (flposX < minVisX) continue;
                if (flposY < 0) continue;
                if (flposX >= resolutionXf_) continue;
                if (flposY >= resolutionYf_) continue;

                wr = posX-flposX;
                wl = 1.0f-wr;
                wb = posY-flposY;
                wt = 1.0f-wb;

                wtl = wl*wt;
                wtr = wr*wt;
                wbl = wl*wb;
                wbr = wr*wb;

                idxFTL = (int)flposY*stepF+ (int)flposX;
                idxFTLY = idxFTL+stepF;



                zImgRes[idxFTL  ] += posZ*wtl;
                assignRes[idxFTL] += wtl;

                zImgRes[idxFTL+1  ] += posZ*wtr;
                assignRes[idxFTL+1] += wtr;

                zImgRes[idxFTLY  ] += posZ*wbl;
                assignRes[idxFTLY] += wbl;

                zImgRes[idxFTLY+1  ] += posZ*wbr;
                assignRes[idxFTLY+1] += wbr;

            }

        }


        for (yl = 0; yl < resolutionY;++yl)
        {
            zImgRes = zImg.ptr<float>(yl);
            assignRes = assignVal.ptr<float>(yl);

            for (xl = 0; xl < resolutionX;++xl)
            {
                if (assignRes[xl] >= minAssignValue_)
                {

                    zImgRes[xl] = (zImgRes[xl]/ assignRes[xl])*scale+baseLevel;

                }
                else
                {
                    zImgRes[xl] = zeroLevel;

                }


            }
        }




    }


    /**
     * @brief Uses the mean over all nearest neighbour pixels
     */
    void ProcessDepthImageNN(const cv::Mat &inD, cv::Mat &zImg, cv::Mat &assignVal, cv::Vec4i &minMax, const float scale, const float baseLevel, const float zeroLevel, const float minVisX = 0.0f)
    {
        int resolutionX = zImg.cols-1;
        int resolutionY = zImg.rows-1;

        minMax[0] = resolutionX;
        minMax[1] = resolutionY;
        minMax[2] = 0;
        minMax[3] = 0;


        UtilsDepthImage::SetToZero(zImg);//.setTo(0);
        UtilsDepthImage::SetToZero(assignVal);//.setTo(0);


        float *zImgRes = zImg.ptr<float>();
        float *assignRes = assignVal.ptr<float>();

        float posX,posY,posZ;

        int idxFTL,step = assignVal.cols;

        int xl,yl;

        const float *ptrD;

        float tx,ty,tz;

        float rx,ry,rz;

        float oz;
        int flPosXi;
        int flPosYi;


        for  (yl = 0; yl < inD.rows;++yl)
        {
            ptrD = inD.ptr<float>(yl);

            for  (xl = 0; xl < inD.cols;++xl)
            {
                oz = ptrD[xl];
                if (oz < minDepthThreshold_) continue;

                tx = ((float(xl)-cx)*fxi)*oz;
                ty = ((float(yl)-cy)*fyi)*oz;
                tz = oz;

                rx = tx*r11+ty*r12+tz*r13+t1;
                ry = tx*r21+ty*r22+tz*r23+t2;
                rz = tx*r31+ty*r32+tz*r33+t3;

                if (std::isnan(tx)) continue;


#ifndef TRANSPOSE_TRANSFORM

                // normal

                posX = (rx-minXVal_) * pixelResolution_;
                posY = (ry-minYVal_) * pixelResolution_;
                posZ = rz;

#else

                // Transposed
                posY = (rx-minXVal_) * pixelResolution_;
                posX = (ry-minYVal_) * pixelResolution_;
                posZ = rz;
#endif


                flPosXi = (int)round(posX);
                flPosYi = (int)round(posY);

                if (flPosXi < 0) continue;
                if (flPosYi < 0) continue;
                if (flPosXi > resolutionX) continue;
                if (flPosYi > resolutionY) continue;
                if (flPosXi < minMax[0] ) minMax[0] = flPosXi;
                if (flPosXi+1 > minMax[2] ) minMax[2] = flPosXi+1;
                if (flPosYi < minMax[1] ) minMax[1] = flPosYi;
                if (flPosYi+1 > minMax[3] ) minMax[3] = flPosYi+1;

                idxFTL = flPosYi*step+ flPosXi;
                zImgRes[idxFTL  ] += posZ;
                assignRes[idxFTL] ++;


            }

        }



    }

    /**
     * @brief Only uses the nearest neighbour with highest z-value
     */
    void ProcessDepthImageMaxNN(const cv::Mat &inD, cv::Mat &zImg, cv::Mat &assignVal, cv::Vec4i &minMax, const float scale, const float baseLevel, const float zeroLevel, const float minVisX = 0.0f)
    {
        int resolutionX = zImg.cols-1;
        int resolutionY = zImg.rows-1;


        minMax[0] = resolutionX;
        minMax[1] = resolutionY;
        minMax[2] = 0;
        minMax[3] = 0;



        zImg.setTo(-10);
        UtilsDepthImage::SetToZero(assignVal);//.setTo(0);


        float *zImgRes = zImg.ptr<float>();
        float *assignRes = assignVal.ptr<float>();

        float posX,posY,posZ;

        int idxFTL,step = assignVal.cols;


        int xl,yl;

        const float *ptrD;

        float tx,ty,tz;

        float rx,ry,rz;

        float oz;
        int flPosXi;
        int flPosYi;


        for  (yl = 0; yl < inD.rows;++yl)
        {
            ptrD = inD.ptr<float>(yl);

            for  (xl = 0; xl < inD.cols;++xl)
            {
                oz = ptrD[xl];
                if (oz < minDepthThreshold_) continue;

                tx = ((float(xl)-cx)*fxi)*oz;
                ty = ((float(yl)-cy)*fyi)*oz;
                tz = oz;

                rx = tx*r11+ty*r12+tz*r13+t1;
                ry = tx*r21+ty*r22+tz*r23+t2;
                rz = tx*r31+ty*r32+tz*r33+t3;

                if (std::isnan(tx)) continue;


#ifndef TRANSPOSE_TRANSFORM

                // normal

                posX = (rx-minXVal_) * pixelResolution_;
                posY = (ry-minYVal_) * pixelResolution_;
                posZ = rz;

#else

                // Transposed
                posY = (rx-minXVal_) * pixelResolution_;
                posX = (ry-minYVal_) * pixelResolution_;
                posZ = rz;
#endif


                flPosXi = (int)round(posX);
                flPosYi = (int)round(posY);

                if (flPosXi < 0) continue;
                if (flPosYi < 0) continue;
                if (flPosXi > resolutionX) continue;
                if (flPosYi > resolutionY) continue;
                if (flPosXi < minMax[0] ) minMax[0] = flPosXi;
                if (flPosXi+1 > minMax[2] ) minMax[2] = flPosXi+1;
                if (flPosYi < minMax[1] ) minMax[1] = flPosYi;
                if (flPosYi+1 > minMax[3] ) minMax[3] = flPosYi+1;

                idxFTL = flPosYi*step+ flPosXi;
                zImgRes[idxFTL  ] = std::max(posZ,zImgRes[idxFTL  ]);
                assignRes[idxFTL]  = 1;


            }

        }


    }

    /**
     * @brief Uses an inverse bilinear interpolation to interpolate the z values of neighbouring pixels
     */
    void ProcessDepthImage(const cv::Mat &inD, cv::Mat &zImg, cv::Mat &assignVal, cv::Vec4i &minMax, const float scale, const float baseLevel, const float zeroLevel, const float minVisX = 0.0f)
    {
        int resolutionX = zImg.cols-1;
        int resolutionY = zImg.rows-1;

        minMax[0] = resolutionX;
        minMax[1] = resolutionY;
        minMax[2] = 0;
        minMax[3] = 0;

        UtilsDepthImage::SetToZero(zImg);//.setTo(0);
        UtilsDepthImage::SetToZero(assignVal);//.setTo(0);


        float *zImgRes = zImg.ptr<float>();
        float *assignRes = assignVal.ptr<float>();

        float posX,posY,posZ;
        float flposX,flposY;

        float wtl,wbl,wtr,wbr;

        float wl,wr,wb,wt;
        int idxFTL,idxFTLY,step = assignVal.cols;


        int xl,yl;

        const float *ptrD;

        float tx,ty,tz;

        float rx,ry,rz;

        float oz;
        int flPosXi;
        int flPosYi;


        for  (yl = 0; yl < inD.rows;++yl)
        {
            ptrD = inD.ptr<float>(yl);

            for  (xl = 0; xl < inD.cols;++xl)
            {
                oz = ptrD[xl];
                if (oz < minDepthThreshold_) continue;

                tx = ((float(xl)-cx)*fxi)*oz;
                ty = ((float(yl)-cy)*fyi)*oz;
                tz = oz;

                rx = tx*r11+ty*r12+tz*r13+t1;
                ry = tx*r21+ty*r22+tz*r23+t2;
                rz = tx*r31+ty*r32+tz*r33+t3;

                if (std::isnan(tx)) continue;



#ifndef TRANSPOSE_TRANSFORM

                // normal
                posX = (rx-minXVal_) * pixelResolution_;
                posY = (ry-minYVal_) * pixelResolution_;
                posZ = rz;

#else

                // Transposed
                posY = (rx-minXVal_) * pixelResolution_;
                posX = (ry-minYVal_) * pixelResolution_;
                posZ = rz;
#endif

                flposX = floor(posX);
                flposY = floor(posY);

                wr = posX-flposX;
                wl = 1.0f-wr;
                wb = posY-flposY;
                wt = 1.0f-wb;

                wtl = wl*wt;
                wtr = wr*wt;
                wbl = wl*wb;
                wbr = wr*wb;

                flPosXi = (int)flposX;
                flPosYi = (int)flposY;

                if (flPosXi < 0) continue;
                if (flPosYi < 0) continue;
                if (flPosXi >= resolutionX) continue;
                if (flPosYi >= resolutionY) continue;


                idxFTL = flPosYi*step+ flPosXi;
                idxFTLY = idxFTL+step;

                if (flPosXi < minMax[0] ) minMax[0] = flPosXi;
                if (flPosXi+1 > minMax[2] ) minMax[2] = flPosXi+1;
                if (flPosYi < minMax[1] ) minMax[1] = flPosYi;
                if (flPosYi+1 > minMax[3] ) minMax[3] = flPosYi+1;


                zImgRes[idxFTL  ] += posZ*wtl;
                assignRes[idxFTL] += wtl;

                zImgRes[idxFTL+1  ] += posZ*wtr;
                assignRes[idxFTL+1] += wtr;

                zImgRes[idxFTLY  ] += posZ*wbl;
                assignRes[idxFTLY] += wbl;

                zImgRes[idxFTLY+1  ] += posZ*wbr;
                assignRes[idxFTLY+1] += wbr;

            }

        }


    }




};


#endif //RGBD_ZIMAGE
