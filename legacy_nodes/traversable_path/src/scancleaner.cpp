#include "scancleaner.h"
#include <ros/ros.h>

ScanCleaner::ScanCleaner()
{
}

void ScanCleaner::clean(sensor_msgs::LaserScan &scan)
{
    vfiter bad_area_start;
    bool in_bad_area = false;

    for (vfiter range_it = scan.ranges.begin(); range_it != scan.ranges.end()-1; ++range_it) {
        if ((*range_it == 0) || (*range_it > MAX_DISTANCE)) {
            if (!in_bad_area) {
                in_bad_area = true;
                bad_area_start = range_it;
            }
        } else {
            if (in_bad_area) {
                in_bad_area = false;
                interpolate(bad_area_start - 1, range_it);
            }
        }

    }
}

void ScanCleaner::interpolate(ScanCleaner::vfiter begin, ScanCleaner::vfiter end)
{
    float a = *begin, b = *end;
    int n = std::distance(begin, end);
    float m = (b - a) / n;

    for (vfiter it = begin+1; it != end; ++it) {
        a += m;
        *it = a;
    }
}
