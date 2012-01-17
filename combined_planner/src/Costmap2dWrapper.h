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

class Costmap2dWrapper : public lib_path::GridMap2d
{
public:
    Costmap2dWrapper( const costmap_2d::Costmap2D* map );

    virtual ~Costmap2dWrapper()
        { /* Nothing to do */ }

    void setDataModel( costmap_2d::Costmap2D* newmap )
        { costmap_ = newmap; }

    /* Inherited from GridMap2d */

    inline int8_t getValue( const unsigned int x, const unsigned int y ) const
        { return (unsigned char)costmap_->getCost( x,y ); }

    inline void setValue( const unsigned int x, const unsigned int y, const int8_t value )
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


    inline bool isFree( const unsigned int x, const unsigned int y ) const {
        int8_t value = getValue( x,y );
        return value >= lower_thres_ && value <= upper_thres_;
    }

    inline bool isOccupied( const unsigned int x, const unsigned int y ) const {
        int8_t value = getValue( x,y );
        return value > upper_thres_ && value < costmap_2d::NO_INFORMATION;
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
        int x, y;
        return point2Cell( p, x,y );
    }

private:
    costmap_2d::Costmap2D* costmap_;
    double lower_thres_;
    double upper_thres_;
};

#endif // COSTMAP2DWRAPPER_H
