////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <algorithm>
#include <Eigen/Core>
using namespace Eigen;

// Project

#include "LowPassFilter.h"

////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////

using namespace std;

//// class LowPassFilter ///////////////////////////////////////////////////////
template <class T>
LowPassFilter<T>::LowPassFilter( size_t size ) :
    mData( size ), mCoeff( size ), mPos( 0 ), mLoaded( false ), mValue( 0 ) {
   SetCoefficients( 1.0f );
}

template <class T>
bool LowPassFilter<T>::IsLoaded() const {
    return mLoaded;
}

template <class T>
void LowPassFilter<T>::Reset() {
    mLoaded = false;
    for ( size_t i = 0; i < GetSize(); ++i )
        mData[i] = 0;
}

template <class T>
void LowPassFilter<T>::SetCoefficients( const T& coeff ) {
    for ( size_t i = 0; i < GetSize(); ++i )
        mCoeff[i] = coeff;
    mCoeffSum = coeff  * (float)mCoeff.size();
}

template <class T>
void LowPassFilter<T>::SetCoefficients( const vector<T> &coeff ) {
    mCoeffSum = 0;
    for ( size_t i = 0; i < GetSize(); ++i ) {
        if ( i < coeff.size())
            mCoeff[i] = coeff[i];
        mCoeffSum += mCoeff[i];
    }
}

template <class T>
T LowPassFilter<T>::Update( const T& newValue ) {
    mPos++;
    if ( mPos >= GetSize()) {
        mPos = 0;
        mLoaded = true;
    }
    mData[mPos] = newValue;

    if ( !mLoaded )
        return newValue;

    mValue = T(0);
    for ( size_t i = mPos; i < GetSize(); ++i ) {
        mValue += ( mData[i] * mCoeff[i - mPos] );
    }
    for ( size_t i = 0; i < mPos; ++i ) {
        mValue += ( mData[i] * mCoeff[ i + GetSize() - mPos ]);
    }
    mValue /= mCoeffSum;
    return mValue;
}




template <class T>
T LowPassFilter<T>::GetMean()
{
    if ( !mLoaded )
        return 0;
    T sum=T(0);
    for ( size_t i = 0; i < GetSize(); ++i ) {
        sum+=mData[i];
    }
    return sum/GetSize();
}

/**
return max value
*/
template <class T>
T LowPassFilter<T>::GetMax()
{
   return *(max_element(mData.begin(),mData.end()));
}


template <class T>
T LowPassFilter<T>::GetMin()
{
   return *(min_element(mData.begin(),mData.end()));
}


template <class T>
bool LowPassFilter<T>::IsInInterval(const T& minVal, const T& maxVal)
{
    if (!IsLoaded()) {
       return false;
    }
    if (GetMax()<maxVal && GetMin()>minVal) {
        return true;
    } else {
        return false;
    }
}



// pre-create template instantiations for double and float
template class LowPassFilter<double>;
template class LowPassFilter<float>;

