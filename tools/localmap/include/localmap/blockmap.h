#ifndef BLOCKMAP_H
#define BLOCKMAP_H

#include <opencv2/core/core.hpp>


/**
 * @brief Local map storage class, the map is robot centered and shifting is done using a block structure
 */
class BlockMap
{
public:


    /**
     * @brief Tests if recentering is required and performs it if required
     */
    void ReCenter(const cv::Point2f &pos);
    /**
     * @brief Actual recentering function
     */
    void CenterMap(const cv::Point2i &newCenterBlock);
    /**
     * @brief Get the block closest to the current position
     */
    cv::Point2i GetClosestBlockPos(const cv::Point2f &pos);
    /**
     * @brief Tests is position is within center blocks
     */
    bool TestSafe(const cv::Point2f &pos);
    /**
     * @brief transforms the current map into base link coordinate system
     */
    void Transform2BaseLink(const cv::Point2d &pos, const double angle);
    /**
     * @brief Converts the world to map coordinates
     */
    cv::Point2d RobotPos2MapPos(cv::Point2d pos);
    cv::Point2f RobotPos2MapPos(cv::Point2f pos);
    /**
     * @brief Sets new center coorditnes
     */
    void UpdateCenter(const cv::Point2f nCenter);
    /**
     * @brief Initialize blockmap
     */
    void Setup();
    /**
     * @brief Set whole map to given value
     */
    void SetMapTo(float val);

    /**
     * @brief Set center blocks to a plane based on vehicle pose
     */
    void SetSafeBlocksTo();


    /**
     * @brief Set blocks around to a plane based on vehicle pose
     */
    void SetSafeAroundRobot();



    /**
     * @brief Set current vehicle pose used for filling safe blocks
     */
    void SetPose(cv::Point3f normal, cv::Point3f pos);

    cv::Point2f center_;
    cv::Point2f origin_;
    int mapResolution_;
    float mapSize_;
    float pixelResolution_;
    int numBlocks_;
    float mapNotVisibleLevel_;
    float mapBaseLevel_;
    float heightScale_;

    cv::Mat currentMap_;
    cv::Mat baseLinkMap_;

    cv::Point3f curNormal_;
    cv::Point3f curPos_;

private:

    cv::Point2f safeMin_;
    cv::Point2f safeMax_;

    cv::Point2f lastPosition_;

    int blockResolution_;

    float blockStep_;
    int safeBlocks_;

    cv::Mat tempMap_;

};


#endif //BLOCKMAP_H
