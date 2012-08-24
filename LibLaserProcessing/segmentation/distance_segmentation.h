/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks, buck
 */

#ifndef DISTANCE_SEGMENTATION_H
#define DISTANCE_SEGMENTATION_H

// C/C++
#include <vector>

// Project
#include <utils/LibLaserProcessing/laser_beam.h>

namespace lib_laser_processing {

/// @todo Extract a parent class to make switching between different algorithms easier

/**
 * @brief Provides a simple distance based segmentation of laser data
 */
class DistanceSegmentation
{
public:

    /**
     * @brief Create and initialize
     * @param threshold Distance threshold
     */
    DistanceSegmentation( float threshold );

    /**
     * @brief Process new laser data
     * @param beams Laser range measurements
     */
    void update( const std::vector<LaserBeam>& beams );

    /**
     * @brief Represents one segment
     */
    struct DistanceSegment
    {
        /// Start index of segment
        unsigned int start;

        /// Number of beams
        unsigned int length;
    };

    /**
     * @brief Interates over all segments
     */
    class Iterator
    {
    public:

        Iterator( DistanceSegmentation* obj ) : seg_idx_(-1), beam_idx_(-1), obj_(obj)
        {}

        /**
         * @brief Select next segment
         * @return False if there is no next segment
         */
        bool nextSegment() {
            ++seg_idx_;
            beam_idx_ = -1;
            return seg_idx_ < (int)(obj_->segms_.size());
        }

        /**
         * @brief Get the length of the current segment
         * @return Number of beams in current segment
         */
        unsigned int length() {
            return obj_->segms_[seg_idx_].length;
        }

        /**
         * @brief Select next beam in current segment
         * @return False if there is no next beam in the current segment
         */
        bool nextBeam() {
            ++beam_idx_;
            return beam_idx_ < obj_->segms_[seg_idx_].length;
        }

        /**
         * @brief Get the selected beam in the current segment
         * @return The laser beam
         */
        LaserBeam& beam() {
            return obj_->beams_[obj_->segms_[seg_idx_].start + beam_idx_];
        }

        /**
         * @brief Reset beam iteration
         */
        void resetBeams() {
            beam_idx_ = 0;
        }

        /**
         * @brief Set status of all beams of the selected segment to invalid
         * @attention This method is not thread safe
         */
        void invalidateSegment() {
            unsigned int old_beam_idx = beam_idx_;
            resetBeams();
            while ( nextBeam())
                beam().valid = false;
            beam_idx_ = old_beam_idx;
        }

    private:
        /// Index of current segment
        int seg_idx_;

        /// Index of current beam
        unsigned int beam_idx_;

        /// We are iteration over the segments of this object
        DistanceSegmentation* obj_;
    };

    /**
     * @brief Create an iterator to iterate over all segments
     * @return The iterator object
     */
    Iterator iterator() {
        return Iterator( this );
    }

private:

    /// Distance threshold [m]
    float threshold_;

    /// All segments
    std::vector<DistanceSegment> segms_;

    /// Laser data
    std::vector<LaserBeam> beams_;

};

} // namespace

#endif // DISTANCE_SEGMENTATION_H
