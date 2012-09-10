/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks
 */

#ifndef PERSON_FILTER_H
#define PERSON_FILTER_H

// C/C++
#include <vector>
#include <list>

// Project
#include <utils/LibLaserProcessing/person_filter/person.h>

namespace lib_laser_processing {

/**
 * @brief Kalman based person filter.
 */
class PersonFilter
{
public:

    /**
     * @brief Represents one hypothesis of the filter.
     */
    class Hypothesis
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// State vector (position, velocity)
        Eigen::Vector4d state;

        /// Covariance matrix
        Eigen::Matrix4d cov;

        /// Initilai position
        Eigen::Vector2d first_seen;

        double person_prob;

        /// Unique id (used for testing)
        long id;
    };


    /**
     * @brief Holds the filter configuration
     */
    class Config
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// Noise added during prediction step (per second)
        Eigen::Matrix4d noise;

        /// Uncertainty of one measurement
        Eigen::Matrix4d measurement_noise;

        /**
         * Maximum position variance. Delete hypothese with a
         * position variance greater than this
         */
        double max_position_var;

        /// Initial covariance matrix
        Eigen::Matrix4d initial_cov;

        double slow_down;
    };

    /**
     * @brief Create filter object with default configuration
     */
    PersonFilter();

    /**
     * @brief Update the filter
     * @param pers Detected persons
     * @param dt Time since last update (seconds)
     */
    void update( const std::vector<Person>& pers, const double dt );

    /**
     * @brief Get all tracked hypotheses
     * @return All hypotheses
     */
    const std::list<Hypothesis>& getHypotheses() const {
        return hypo_;
    }

    /**
     * @brief Update configuration
     * @param c New configuration
     */
    void setConfig( const Config& c ) {
        config_ = c;
    }

private:

    /**
     * @brief Try to find the best matching hypothese for a given position
     */
    Hypothesis* getMatchingHypo( const Person& p, const double dt );

    /**
     * @brief Create a new hypothesis
     */
    Hypothesis createNewHypo( const Person& p );

    /**
     * @brief Return probability of x relative to state and cov
     */
    double positionProbability( const Eigen::Vector4d& x,
                                const Eigen::Vector4d& state,
                                const Eigen::Matrix4d& cov );

    /// All hypotheses
    std::list<Hypothesis> hypo_;

    /// Filter configuration
    Config config_;

    /// Used to generate unique IDs
    long last_id_;
};

} // namespace

#endif // PERSON_FILTER_H
