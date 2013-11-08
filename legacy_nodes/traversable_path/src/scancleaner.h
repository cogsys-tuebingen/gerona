#ifndef SCANCLEANER_H
#define SCANCLEANER_H

#include <vector>
#include <sensor_msgs/LaserScan.h>

/**
 * @brief Cleans a laserscan by interpolating invalid values.
 *
 * The ibeo LUX laserscanner produces a lot of outlier. Theses outliers are mostly single points with distances
 * > 100m which is obviously wrong if the scanner is tilted on the ground and the surrounding points have distances
 * of less then 2m.
 *
 * This class provides a mthod to detect such worng points and replace them by points that are interpolated by the valid
 * neighbours.
 */
class ScanCleaner
{
public:
    typedef std::vector<float>::iterator vfiter;

    //! Maximum distance which assumed to be correct (10m is sufficient, since the ourliers have values > 100).
    static const float MAX_DISTANCE = 10.0;

    ScanCleaner();

    static void clean(sensor_msgs::LaserScan &scan);

private:
    static void interpolate(vfiter begin, vfiter end);
};

#endif // SCANCLEANER_H
