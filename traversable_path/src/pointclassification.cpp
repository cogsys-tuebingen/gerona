#include "pointclassification.h"

bool PointClassification::operator==(PointClassification p)
{
    return (this->classification_ == p.classification_)
            && (this->obstacle_value_ == p.obstacle_value_);
}

bool PointClassification::operator!=(PointClassification p)
{
    return !operator ==(p);
}


classification_t PointClassification::classification()
{
    return classification_;
}

int PointClassification::obstacle_value()
{
    return obstacle_value_;
}


void PointClassification::setFlag(classification_t flag)
{
    classification_ |= flag;
    obstacle_value_ += weightByFlag(flag);
}

int PointClassification::weightByFlag(classification_t flag)
{
    switch (flag) {
    case FLAG_DIFF_RANGE_OVER_LIMIT:
        return WEIGHT_DIFF_RANGE_OVER_LIMIT;
    case FLAG_DIFF_INTENSITY_OVER_LIMIT:
        return WEIGHT_DIFF_INTENSITY_OVER_LIMIT;
    default:
        ROS_ERROR("PointClassification: Unknown flag %d", flag);
        return 0;
    }
}

bool PointClassification::isTraversable()
{
    return obstacle_value_ < OBSTACLE_VALUE_LIMIT;
}
