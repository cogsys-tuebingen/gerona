#ifndef POINTCLASSIFICATION_H
#define POINTCLASSIFICATION_H

#include <stdint.h>
#include <ros/ros.h>

//! Type of the point classification flags.
typedef uint8_t classification_t;

/**
 * @brief Classification of a point.
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class PointClassification {
private:
    static const short WEIGHT_DIFF_RANGE_OVER_LIMIT = 80;
    static const short WEIGHT_DIFF_INTENSITY_OVER_LIMIT = 60;
    static const short WEIGHT_DIFF_INTENSITY_NEIGHBOUR = 20;
    static const short WEIGHT_DIFF_RANGE_NEIGHBOUR = 20;
    static const short WEIGHT_UNTRAVERSABLE_IN_PAST_SCANS = 100;

    //! Classification flags. One flag per bit.
    classification_t classification_;
    //! Obstacle value (= sum of flag-weights)
    short obstacle_value_;

    //! Get the weight of the specified flag.
    short weightByFlag(classification_t flag);

public:
    static const short OBSTACLE_VALUE_LIMIT = 100;
    // flags
    static const classification_t FLAG_NONE = 0;
    static const classification_t FLAG_DIFF_RANGE_OVER_LIMIT = 1;
    static const classification_t FLAG_DIFF_INTENSITY_OVER_LIMIT = 2;
    static const classification_t FLAG_DIFF_RANGE_NEIGHBOUR = 4;
    static const classification_t FLAG_DIFF_INTENSITY_NEIGHBOUR = 8;
    static const classification_t FLAG_UNTRAVERSABLE_IN_PAST_SCANS = 16;

    PointClassification():
            classification_(0),
            obstacle_value_(0)
    {}

    bool operator==(PointClassification p);
    bool operator!=(PointClassification p);

    /**
     * @brief Get classification flags of the point.
     *
     * Each bit represents a specific flag. Check for a flag using bit operators:
     * <code>
     *   if (foo.classification() & PointClassification::FLAG_DIFF_RANGE_OVER_LIMIT) { flag is set... }
     * </code>
     *
     * @return Classification flags.
     */
    classification_t classification();

    /**
     * @brief Get obstacle value of the point.
     *
     * If value is greater than PointClassification::OBSTACLE_VALUE_LIMIT, the point is classified as obstacle,
     * otherwise as traversable.
     * @return Obstacle value.
     */
    short obstacle_value();

    /**
     * @brief Set a classification flag.
     *
     * The obstacle value of the point will automaticly be increased depending of the weight of this flag.
     */
    void setFlag(classification_t flag);

    //! True, if the point is traversable, false if not.
    bool isTraversable();
};

#endif // POINTCLASSIFICATION_H
