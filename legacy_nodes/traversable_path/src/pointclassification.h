#ifndef POINTCLASSIFICATION_H
#define POINTCLASSIFICATION_H

#include <stdint.h>
#include <bitset>

//! Type of the point classification flags.
typedef std::bitset<2> classification_flags_t;

/**
 * @brief Classification of a point.
 *
 * Represents the classification of a scanned point. In the past there was a classification system using flags for
 * several features. This is obsolete, but the structure of this flag-system mostly remains for the moment as it might
 * be usefull for debugging (e.g to visualise the reason for which a point became untraversable).
 * The obstacle value mechanism also remains for now, but is currently of no use.
 *
 * The new system works as follows:
 * There are two classes 'traversable' and 'untraversable' plus an additional 'unknown'-label.
 * The traversability of a point is no longer defined by the flags but the class is simply set by setClass().
 * The decision to which class the point belongs is no more done by this class but by an external machine learning
 * classifier.
 *
 *
 * ----- old (obsolete) description of the flag system
 * Represents the classification of a scanned point. There are different features which influence the final
 * classification with different weights.
 *
 * Each of these features has a own flag, defined as constant of the class (PointClassification::FLAG_*).
 * The weights of this features are also defined as constants but private and thus can not be accessed from outside.
 *
 * Thus the classification for a single point contains a set of flags and an "obstacle value" which is simply the sum
 * of the weights of all detected features.
 * To set a feature as detected, use setFlag(). This will automaticly increase the obstacle value. You can not change
 * the obstacle value directly.
 *
 * A point is classified as traversable if the obstacle value does not exceed PointClassification::OBSTACLE_VALUE_LIMIT.
 * To check this, use the method isTraversable().
 * -----
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class PointClassification {
public:
    //! Enum of the possible classes.
    enum Class
    {
        UNKNOWN,
        UNTRAVERSABLE,
        TRAVERSABLE
    };


    //! The point is classified as untraversable if the obstacle value exceeds this limit.
    static const short OBSTACLE_VALUE_LIMIT = 100;
    // flags
    static const uint8_t FLAG_UNTRAVERSABLE_IN_PAST_SCANS = 0;
    static const uint8_t FLAG_CLASSIFIED_AS_UNTRAVERSABLE = 1;
    // note: when you add new flags, you also have to increase the size of the bitset (classification_flags_t) above.

    PointClassification():
            obstacle_value_(0),
            class_(UNKNOWN)
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
    classification_flags_t getFlags() const;

    /**
     * @brief Get obstacle value of the point.
     *
     * If value is greater than PointClassification::OBSTACLE_VALUE_LIMIT, the point is classified as obstacle,
     * otherwise as traversable.
     * @return Obstacle value.
     * @deprecated See class description.
     */
    short obstacle_value() const;

    /**
     * @brief Set a classification flag.
     *
     * The Flags have no functional meaning for the moment. They may be used for debugging ("tell me why this point
     * is marked as untraversable") or otherwise removed in the future.
     * //The obstacle value of the point will automaticly be increased depending of the weight of this flag.
     */
    void setFlag(uint8_t flag);

    //! Set class of this point.
    void setClass(Class c)
    {
        class_ = c;
    }

    Class getClass() const
    {
        return class_;
    }


private:
    static const short WEIGHT_UNTRAVERSABLE_IN_PAST_SCANS = OBSTACLE_VALUE_LIMIT;
    static const short WEIGHT_CLASSIFIED_AS_UNTRAVERSABLE = OBSTACLE_VALUE_LIMIT;

    //! Classification flags. One flag per bit.
    classification_flags_t classification_;
    //! Obstacle value (= sum of flag-weights)
    short obstacle_value_;

    //! The class which is assigned to this point. See enum Class for possible values.
    Class class_;

    //! Get the weight of the specified flag.
    /** \deprecated */
    static short weightByFlag(uint8_t flag);

};

#endif // POINTCLASSIFICATION_H
