/**
 * @file Costmap2dWrapper.h
 * @date Jan 2012
 * @author marks
 */

#ifndef COSTMAP2DWRAPPER_H
#define COSTMAP2DWRAPPER_H

// ROS
#include <costmap_2d/costmap_2d.h>

// Workspace
#include <utils/LibPath/common/GridMap2d.h>

/**
 * @class Costmap2dWrapper
 * @brief A ROS Costmap2d wrapper.
 *
 * This class makes it possible to use a ROS costmap implementation as input for
 * algorithms that use the ROS independend map interface from utils/LibPath.
 *
 * @attention At the current state not all member functions of this class are
 *      reviewed and fully tested. Use it with care!
 */
class Costmap2dWrapper : public lib_path::GridMap2d
{
public:

    /**
     * @brief Create a ROS Costmap2d wrapper.
     * The lower threshold defaults to 20, the upper one to 200.
     * @param map The wrapped costmap.
     */
    Costmap2dWrapper( costmap_2d::Costmap2D* map );

    virtual ~Costmap2dWrapper()
        { /* Nothing to do */ }

    /**
     * @brief Setter for the wrapped costmap object.
     * @attention There is no local copy of the costmap! The given pointer should be
     *      valid as long as this wrapper is in use.
     * @param newmap Points to the new underlying costmap.
     */
    void setCostmap( costmap_2d::Costmap2D* newmap )
        { costmap_ = newmap; }

    /**
     * @brief Set the lower threshold.
     * Every cell with a value less or equal to the given threshold is obstacle free.
     * @param thres The new lower threshold
     */
    void setLowerThreshold( const uint8_t thres )
        { lower_thres_ = thres; }

    /**
     * @brief Set the upper threshold.
     * Every cell with a value greater or equal to this threshold represents an obstacle.
     * This is not true for cell thats contain no information (value 255).
     * @param thres The new upper threshold.
     */
    void setUpperThreshold( const uint8_t thres )
        { upper_thres_ = thres; }


    /* Abstract functions inherited from GridMap2d */

    inline uint8_t getValue( const unsigned int x, const unsigned int y ) const
        { return (unsigned char)costmap_->getCost( x,y ); }

    inline void setValue( const unsigned int x, const unsigned int y, const uint8_t value )
        { return costmap_->setCost( x, y, (unsigned char)value ); }

    unsigned int getWidth() const
        { return costmap_->getSizeInCellsX(); }

    unsigned int getHeight() const
        { return costmap_->getSizeInCellsY(); }

    double getResolution() const
        { return costmap_->getResolution(); }

    lib_path::Point2d getOrigin() const
        { return lib_path::Point2d( costmap_->getOriginX(), costmap_->getOriginY()); }

    void setOrigin( const lib_path::Point2d& p ) /// @todo Think about this!
        { costmap_->updateOrigin( p.x, p.y ); }


    inline bool isFree( const unsigned int x, const unsigned int y ) const
        { return getValue( x,y ) <= lower_thres_; }

    inline bool isOccupied( const unsigned int x, const unsigned int y ) const {
        uint8_t value = getValue( x,y );
        return value >= upper_thres_ && value < costmap_2d::NO_INFORMATION;
    }

    inline bool isNoInformation( const unsigned int x, const unsigned int y ) const
        { return costmap_->getCost( x,y ) == costmap_2d::NO_INFORMATION; }

    inline bool point2Cell( const lib_path::Point2d& p, unsigned int& x, unsigned int& y ) const
        { return costmap_->worldToMap( p.x, p.y, x, y ); }

    inline void cell2point( const int x, const int y, lib_path::Point2d& p ) const
        { costmap_->mapToWorld( x, y, p.x, p.y ); }

    inline bool isInMap( const int x, const int y ) const
        { return !( x < 0 || x >= getWidth() || y < 0 || y > getHeight()); }

    inline bool isInMap( const lib_path::Point2d& p ) const {
        unsigned int x, y;
        return point2Cell( p, x, y );
    }

private:

    /// Points to the underlying costmap
    costmap_2d::Costmap2D* costmap_;

    /// Lower threshold. Every cell with a value less or equal this threshold is free
    uint8_t lower_thres_;

    /** Upper threshold. Every cell with a value greater or equal to this threshold is occupied
     * (or it contains no information). */
    uint8_t upper_thres_;
};

#endif // COSTMAP2DWRAPPER_H
