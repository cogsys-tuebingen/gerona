/**
  a general ring buffer

  */
#ifndef RINGBUFFER_H
#define RINGBUFFER_H

template <class T>
class RingBuffer {
    * Constructor.
    *
    * @param size Size of the sample storage.
    */
   RingBuffer( size_t size )
        :data_(size),pos(-1),loaded_(false) {}

   /**
    * Process a new sample and return the filtered value.
    *
    * @param newValue The new sample.
    * @return The filtered value.
    */
   T update( const T& new_value ) {
       mData[pos_] = newValue;
       pos_++;
       if ( pos_ >= GetSize()) {
           pos_ = 0;
           loaded_ = true;
       }
   }

   /**
    * Is to sample storage initialized?
    *
    * @return true if the sample storage is initialized and the filtered data
    * is valid. Discard all filtered data if the filter is not loaded.
    */
   bool isLoaded() const {return loaded_;}

   /**
    * Unloads the filter (remove all stored samples).
    */
   void reset() {pos_=0;loaded_=false_;}

   /**
    * Returns the size of the sample storage.
    *
    * @return The size of the sample storage.
    */
   size_t getSize() const { return data_.size(); }

   /**
     return the samples in order with samples[0] the oldest sample and samples[n-1] the most recent
     @return false if buffer is not loaded yet
     */
   bool getSamples(std::vector<T>& samples) const {
       if (!loaded_) {return false;}
       int idx=pos;
       for (int i=0;i<getSize();++i) {
           if (idx>=getSize()) {
               idx=0;
           }
           samples[i]=data_[idx++];
       }
   }


private:
   /** Sample storage. */
   std::vector<T> data_;
   /** Points to the uptodate sample */
   int pos_;
   /** Filter loaded? */
   bool loaded_;
};



#endif // RINGBUFFER_H
