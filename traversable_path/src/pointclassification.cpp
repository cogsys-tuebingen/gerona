#include "pointclassification.h"
#include <ros/ros.h>

bool PointClassification::operator==(PointClassification p)
{
    return (this->classification_ == p.classification_)
            && (this->obstacle_value_ == p.obstacle_value_);
}

bool PointClassification::operator!=(PointClassification p)
{
    return !operator ==(p);
}


classification_flags_t PointClassification::getFlags() const
{
    return classification_;
}

short PointClassification::obstacle_value() const
{
    return obstacle_value_;
}


void PointClassification::setFlag(uint8_t flag)
{
    classification_.set(flag);
    obstacle_value_ += weightByFlag(flag);
}

short PointClassification::weightByFlag(uint8_t flag)
{
    switch (flag) {
//    case FLAG_DIFF_RANGE_OVER_LIMIT:
//        return WEIGHT_DIFF_RANGE_OVER_LIMIT;
//    case FLAG_DIFF_INTENSITY_OVER_LIMIT:
//        return WEIGHT_DIFF_INTENSITY_OVER_LIMIT;
//    case FLAG_DIFF_INTENSITY_NEIGHBOUR:
//        return WEIGHT_DIFF_INTENSITY_NEIGHBOUR;
//    case FLAG_DIFF_RANGE_NEIGHBOUR:
//        return WEIGHT_DIFF_RANGE_NEIGHBOUR;
//    case FLAG_HEIGHT_OVER_LIMIT:
//        return WEIGHT_HEIGHT_OVER_LIMIT;
//    case FLAG_VARIANCE_OVER_LIMIT:
//        return WEIGHT_VARIANCE_OVER_LIMIT;
    case FLAG_UNTRAVERSABLE_IN_PAST_SCANS:
        return WEIGHT_UNTRAVERSABLE_IN_PAST_SCANS;
    case FLAG_CLASSIFIED_AS_UNTRAVERSABLE:
        return WEIGHT_CLASSIFIED_AS_UNTRAVERSABLE;
    default:
        ROS_ERROR("PointClassification: Unknown flag %d", flag);
        return 0;
    }
}

//bool PointClassification::isTraversable() const
//{
//    return obstacle_value_ < OBSTACLE_VALUE_LIMIT;
//}
