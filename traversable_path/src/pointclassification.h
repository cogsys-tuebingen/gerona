#ifndef POINTCLASSIFICATION_H
#define POINTCLASSIFICATION_H

#include <stdint.h>
#include <ros/ros.h>

typedef uint8_t classification_t;

class PointClassification {
private:
    static const short OBSTACLE_VALUE_LIMIT = 100;
    static const short WEIGHT_DIFF_RANGE_OVER_LIMIT = 100;
    static const short WEIGHT_DIFF_INTENSITY_OVER_LIMIT = 60;
    static const short WEIGHT_DIFF_INTENSITY_NEIGHBOUR = 40;

    classification_t classification_;
    short obstacle_value_;

    short weightByFlag(classification_t flag);

public:
    // flags
    static const classification_t FLAG_NONE = 0;
    static const classification_t FLAG_DIFF_RANGE_OVER_LIMIT = 1;
    static const classification_t FLAG_DIFF_INTENSITY_OVER_LIMIT = 2;
    static const classification_t FLAG_DIFF_INTENSITY_NEIGHBOUR = 4;

    PointClassification():
            classification_(0),
            obstacle_value_(0)
    {}

    bool operator==(PointClassification p);
    bool operator!=(PointClassification p);

    classification_t classification();
    short obstacle_value();

    void setFlag(classification_t flag);
    bool isTraversable();
};

/*
struct PointClassification {
    bool traversable_by_range;
    bool traversable_by_intensity;

    bool operator==(PointClassification p) {
        return (this->traversable_by_intensity == p.traversable_by_intensity)
                && (this->traversable_by_range == p.traversable_by_range);
    }

    bool operator!=(PointClassification p) {
        return !operator ==(p);
    }

    bool isTraversable() {
        return traversable_by_intensity && traversable_by_range;
    }
};
*/

#endif // POINTCLASSIFICATION_H
