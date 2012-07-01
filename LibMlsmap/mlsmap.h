#ifndef MLSMAP_H
#define MLSMAP_H

#include <vector>
#include "field.h"
#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

using namespace std;

// MLS map of 16x16 Fields with 256x256 Patches per Field

template <class CellT>
class MLSmap
{
public:
    MLSmap(int cellSize, int gapSize);
    ~MLSmap();
    bool isField(int x, int y); // tests if Field at coordinates x,y exists
    bool isValidCoord(int x, int y); // tests if Coordinates are within maximum sized map
    CellT* getCell(int x, int y); // get patch by 2d Index
    CellT* getCellByCoord(double x, double y); // get patch by Coordinates in meters
    void update(Eigen::Vector3d & point, double variance); // coordinates in meters, variance in m^2
    void update(const vector<Eigen::Vector3d> & points, const vector<double> & variances);
    void merge(MLSmap mlsmap);
    std::vector<Field<CellT>*> getFields();
    int getCellSize();
    int getGapSize();
    void setCellSize(int cellSize);
    void setGapSize(int gapSize);
    unsigned int getFieldNumber();
    int toIndex(double x);
    void getBounds(double &minX, double &maxX, double &minY, double &maxY);
    void setField( Field<CellT>* f, int f_index );
    void getCoord( int field_index, int cell_index, double& x, double& y ) const;

private:
    int mCellSize; // Size of Patch in cm
    int mGapSize; // minimum distance between two surfaces in the same patch (in cm)
    const int mHalfMapSize; //don't change this value. The hash functions getFieldIndex() and getCellIndex depend on this
    const int mMetersToCentimeters;

    bool mExtremaInit;
    double mMaxX;
    double mMinX;
    double mMaxY;
    double mMinY;

    std::vector<Field<CellT>*> mFields; //vector von 256 fields

    int getFieldIndex(int x, int y); // bildet von 2d Map-Koordinaten auf Indizes des lin. Field vector ab.

    int getCellIndex(int x, int y); // bildet von 2d Patchindizes auf linearen Patch vector ab

    void addField(int x,int y); // allocates new Field at coordinates x,y
    Field<CellT>* getField(int x,int y); // return: Field* auf Field an Indizes x,y (x,y < 16, da 16x16 Fields maximal)
    Field<CellT>* getFieldByCoord(double x, double y);
};

template <class CellT>
MLSmap<CellT>::MLSmap(int cellSize, int gapSize) : mCellSize(cellSize), mGapSize(gapSize), mHalfMapSize(2048), mMetersToCentimeters(100), mExtremaInit(false)
{
    // Fieldpointer ausnullen
    mFields.resize(256,0);
}

template <class CellT>
        MLSmap<CellT>::~MLSmap()
{
    for(unsigned int i = 0; i < mFields.size(); i++){
        delete mFields[i];
    }
}

template <class CellT>
void MLSmap<CellT>::getBounds(double &minX, double &maxX, double &minY, double &maxY)
{
    minX = mMinX;
    maxX = mMaxX;
    minY = mMinY;
    maxY = mMaxY;
}

template <class CellT>
Field<CellT>* MLSmap<CellT>::getField(int x, int y)
{
    return mFields[ getFieldIndex(x,y) ];
}

template <class CellT>
        Field<CellT>* MLSmap<CellT>::getFieldByCoord(double x, double y)
{
    return getField(floor(x/mCellSize), floor(y/mCellSize));
}

// todo: was soll passieren, wenn auf patch zugegriffen wird, der nicht allokiert ist?
// vor Zugriff muss immer ueber einen Aufruf von isField getestet werden, ob Field existiert
template <class CellT>
CellT* MLSmap<CellT>::getCell(int x, int y)
{
    Field<CellT>* f = getField(x, y);
    return (&(f->cells[getCellIndex(x,y)]));
}

template <class CellT>
int MLSmap<CellT>::getFieldIndex(int x, int y)
{
    // Should never use more than 16 byte. See binary.cpp
    return (((x>>8)&0xf)<<4) + ((y>>8)&0xf);
}

template <class CellT>
bool MLSmap<CellT>::isField(int x, int y)
{
    return (isValidCoord(x,y) && getField(x,y) != 0);
}

template <class CellT>
bool MLSmap<CellT>::isValidCoord(int x, int y)
{
    return (x < 2*mHalfMapSize && x >= 0 && y < 2*mHalfMapSize && y >= 0);
}

template <class CellT>
void MLSmap<CellT>::addField(int x, int y)
{
    mFields[ getFieldIndex(x,y) ] = new Field<CellT>();
}

template <class CellT>
void MLSmap<CellT>::setField( Field<CellT> *f, int f_index )
{
    mFields[f_index] = f;
}

template <class CellT>
int MLSmap<CellT>::getCellIndex(int x, int y)
{
       return ((x & 0xff) << 8) + (y & 0xff);
}

template <class CellT>
CellT* MLSmap<CellT>::getCellByCoord(double x, double y)
{
    return getCell(floor(x/mCellSize), floor(y/mCellSize));
}

template <class CellT>
void MLSmap<CellT>::getCoord( int field_index, int cell_index, double &x, double &y ) const
{
    x = (double)((cell_index >> 8) & 0xFF) * mCellSize / 100.0;
    y = (double)(cell_index & 0xFF) * mCellSize / 100.0;
    x += (double)((((field_index & 0xF0) << 4) - mHalfMapSize)*mCellSize )/100.0;
    y += (double)((((field_index & 0x0F) << 8) - mHalfMapSize)*mCellSize )/100.0;
}

template <class CellT>
int MLSmap<CellT>::toIndex(double x)
{
    return floor(x * mMetersToCentimeters / mCellSize) + mHalfMapSize; // add half length of map to center (0,0)
}

template <class CellT>
        void MLSmap<CellT>::update(Eigen::Vector3d & point, double variance)
{
    int x = floor(point[0] * mMetersToCentimeters / mCellSize) + mHalfMapSize; // add half length of map to center (0,0)
    int y = floor(point[1] * mMetersToCentimeters / mCellSize) + mHalfMapSize;
    if (isValidCoord(x,y)){
        if(!mExtremaInit) {
            mExtremaInit = true;
            mMinX = point[0];
            mMaxX = point[0];
            mMinY = point[1];
            mMaxY = point[1];
        } else {
            if(point[0] > mMaxX) mMaxX = point[0];
            if(point[0] < mMinX) mMinX = point[0];
            if(point[1] > mMaxY) mMaxY = point[1];
            if(point[1] < mMinY) mMinY = point[1];
        }
        if (!isField(x, y)){
            addField(x, y);
        }

        CellT* cell = getCell(x, y);
        short int height = round(point[2] * mMetersToCentimeters);
        short int shortVariance = round(variance * 1000000); //convert variance from m^2 to mm^2
        cell->include(height, shortVariance, mGapSize);
    }

}

template <class CellT>
void MLSmap<CellT>::update(const vector<Eigen::Vector3d> & points, const vector<double> & variances)
{
    for (int i = 0; i < points.size(); i++){
        update(points[i], variances[i]);
    }
}

template <class CellT>
std::vector<Field<CellT>*> MLSmap<CellT>::getFields(){
    return mFields;
}

template <class CellT>
int MLSmap<CellT>::getCellSize(){
    return mCellSize;
}

template <class CellT>
int MLSmap<CellT>::getGapSize(){
    return mGapSize;
}

template <class CellT>
void MLSmap<CellT>::setCellSize( int cellSize ){
    mCellSize = cellSize;
}

template <class CellT>
void MLSmap<CellT>::setGapSize( int gapSize ){
    mGapSize = gapSize;
}

template <class CellT>
unsigned int MLSmap<CellT>::getFieldNumber() {
  unsigned num=0;
  for (unsigned i=0;i<mFields.size();++i) {
    if (mFields[i]) ++num;
  }
  return num;
}


#endif // MLSMAP_H
