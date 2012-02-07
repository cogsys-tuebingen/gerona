/*
 * LowPassFilter.h
 *
 *  Created on: Jun 10, 2010
 *      Author: marks
 */

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>
#include <stddef.h>

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Implements a simple lowpass filter.
 */

template <class T>
class LowPassFilter {

public:
    /**
     * Constructor.
     *
     * @param size Size of the sample storage.
     */
    LowPassFilter( size_t size );

    /**
     * Process a new sample and return the filtered value.
     *
     * @param newValue The new sample.
     * @return The filtered value.
     */
    T Update( const T& newValue );

    /**
     * Returns the current filtered value.
     *
     * @return The current value of the filter.
     */
    T GetValue() const { return mValue; }

    /**
     * Is to sample storage initialized?
     *
     * @return true if the sample storage is initialized and the filtered data
     * is valid. Discard all filtered data if the filter is not loaded.
     */
    bool IsLoaded() const;

    /**
     * Unloads the filter (remove all stored samples).
     */
    void Reset();

    /**
     * Returns the size of the sample storage.
     *
     * @return The size of the sample storage.
     */
    size_t GetSize() const { return mData.size(); }

    /**
      return the samples in order with samples[0] the oldest sample and samples[n-1] the most recent
      */
    void GetSamples(std::vector<T>& samples) const;

    /**
     * Sets the filter coefficients.
     *
     * @param coeff The filter coefficients.
     */
    void SetCoefficients( const std::vector<T>& coeff );

    /**
     * Sets all coefficients to the given value.
     *
     * @param coeff New coefficient.
     */
    void SetCoefficients( const T& coeff );

    /**
      return mean
      */
    T GetMean();

    bool IsInInterval(const T& minVal, const T& maxVal);
    /**
      return max
      */
    T GetMax();

    /**
      return min
      */
    T GetMin();

private:
    /** Sample storage. */
    std::vector<T> mData;
    /** Filter coefficients. */
    std::vector<T> mCoeff;
    /** Coefficient sum */
    T mCoeffSum;
    /** Points to the uptodate sample */
    int mPos;
    /** Filter loaded? */ 
    bool mLoaded;
    /** Current filtered value. */
    T mValue;
};

#endif // LOWPASSFILTER_H
