/*
 * DataListener.cpp
 *
 *  Created on: Feb 20, 2010
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Project
#include "DataListener.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

//// class DataOwner //////////////////////////////////////////////////////////

DataOwner::DataOwner() : mValidData( false ) {
}

void DataOwner::addDataListener( DataListener * listener ) {
    for ( size_t i = 0; i < mDataLis.size(); ++i ) {
        if ( mDataLis[i] == listener )
            return;
    }
    mDataLis.push_back( listener );
}

void DataOwner::removeDataListener( DataListener * listener ) {
    DataListenerVector::iterator iter = mDataLis.begin();
    for ( ; iter != mDataLis.end(); iter++ ) {
        if ( *iter == listener ) {
            mDataLis.erase( iter );
            return;
        }
    }
}

void DataOwner::fireDataChanged() {
    DataChangeEvent event;
    event.eventType = DataChangeEvent::NEW_DATA;
    event.owner = this;
    DataListenerVector::iterator iter = mDataLis.begin();
    for ( ; iter != mDataLis.end(); iter++ ) {
        (*iter)->dataChanged( event );
    }
}

void DataOwner::fireStateChanged() {
    DataChangeEvent event;
    event.eventType = DataChangeEvent::STATE;
    event.owner = this;
    DataListenerVector::iterator iter = mDataLis.begin();
    for ( ; iter != mDataLis.end(); iter++ ) {
        (*iter)->dataChanged( event );
    }
}

void DataOwner::setValidData( bool valid ) {
    if ( valid != mValidData ) {
        mValidData = valid;
        fireStateChanged();
    }
}

//// class DataChangeFlag /////////////////////////////////////////////////////

DataChangeFlag::DataChangeFlag() : mChangeFlag( false ) {
}

void DataChangeFlag::dataChanged( const DataChangeEvent &event ) {
    mChangeFlag = true;
}

