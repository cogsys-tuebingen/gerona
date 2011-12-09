#ifndef ACTMULTIARRAYPROXY_H
#define ACTMULTIARRAYPROXY_H


#include <libplayerc++/playerc++.h>


class PLAYERCC_EXPORT ActMultiArrayProxy : public ClientProxy
{
  private:

    void Subscribe(uint32_t aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_m io_t *mDevice;

  public:

    AioProxy (PlayerClient *aPc, uint32_t aIndex=0);
    ~AioProxy();

    /// Accessor function
    uint32_t GetCount() const { return(GetVar(mDevice->voltages_count)); };

    /// Accessor function
    double GetVoltage(uint32_t aIndex)  const
      { return(GetVar(mDevice->voltages[aIndex])); };

    /// Set the output voltage
    void SetVoltage(uint32_t aIndex, double aVoltage);

    /// AioProxy data access operator.
    ///    This operator provides an alternate way of access the actuator data.
    ///    For example, given a @p AioProxy named @p bp, the following
    ///    expressions are equivalent: @p ap.GetVoltage(0) and @p ap[0].
    double operator [](uint32_t aIndex) const
      { return GetVoltage(aIndex); }

};





#endif // ACTMULTIARRAYPROXY_H
