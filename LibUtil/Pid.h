/*****************************************************************************
 * @file   PID.H
 * @brief  Implements a PID controller.
 * @author Hannes Becker (hbecker),
 *         Sebastian Scherer (sscherer),
 *         Deniz Bahadir (bahadir)
 *****************************************************************************/
#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED

/*===========================================================================*
 * INCLUDES (C/C++ Standard Library and other external libraries)
 *===========================================================================*/
#include <cmath>
#include <algorithm>

/*===========================================================================*
 * NAMESPACE, CLASS and TYPE DECLARATIONS
 *===========================================================================*/
namespace attempto {

/*****************************************************************************
 * @brief This class implements a PID controller
 *
 * @author Hannes Becker (hbecker),
 *         Sebastian Scherer (sscherer),
 *         Deniz Bahadir (bahadir)
 *****************************************************************************/
template <class T>
class PID
{
  // --- public constructors ------------------------------------------------
  public:
  /**
   * @brief constructor.
   *
   * @param tW    the desired setPoint.
   * @param fKp   the proportional part's coefficient.
   * @param fKi   the integral part's coefficient.
   * @param fKd   the derivative part's coefficient.
   */
  inline PID(const T     tW  = 0.0f,
             const float fKp = 0.0f,
             const float fKi = 0.0f,
             const float fKd = 0.0f);


  // --- public methods -----------------------------------------------------
  /**
   * @brief Calculates the desired regulation.
   *
   * @param rtActVal The actual value that will be regulated.
   * @param fTime    The time interval between two regulation steps. (If not
   *                 set defaults to @c 1.)
   * @return The PID controller's output.
   */
  inline T regulate(const T&    rtActualVal,
                    const float fTime = 1.0f);

  /**
   * @brief Sets upper bounds for the correction values.
   */
  inline void setLimits(const T& rtLimP,
                        const T& rtLimI,
                        const T& rtLimD);

  /**
   * @brief Enables or disables upper bounds for the correction values.
   */
  inline void activateLimits(const bool bOn);

  /**
   * @brief Returns the set point.
   * @return The set point.
   */
  inline T getW() const;

  /**
   * @brief Sets the set point.
   * @param tW The desired set point.
   */
  inline void setW(const T& rtW);


  // --- private member variables -------------------------------------------
  private:

    // The set point
    T m_tW;
    // Difference between set point and actual value.
    T m_tCurXW;
    // Last difference.
    T m_tLastXW;
    // Sum over all previous differences.
    T m_tSumXW;

    // The PID controller's coefficients
    float m_fKp;
    float m_fKi;
    float m_fKd;

    // A limit for each part of the controller.
    T m_tLimP;
    T m_tLimI;
    T m_tLimD;
    bool m_bLimit;
};


/*===========================================================================*
 * INLINE DEFINITIONS
 *===========================================================================*/

//----------------------------------------------------------------------
// PID INLINE DEFINITIONS
//----------------------------------------------------------------------

template<class T> inline PID<T>::PID(const T     tW,
                                     const float fKp,
                                     const float fKi,
                                     const float fKd)
  : m_tW(tW),
    m_tCurXW(0.0f),
    m_tLastXW(0.0f),
    m_tSumXW(0.0f),
    m_fKp(fKp),
    m_fKi(fKi),
    m_fKd(fKd),
    m_tLimP(0.0f),
    m_tLimI(0.0f),
    m_tLimD(0.0f),
    m_bLimit(false)
{
}


template<class T> inline T PID<T>::regulate(const T&    rtActualVal,
                                            const float fTime)
{
  // Adapt to new values
  m_tLastXW = m_tCurXW;
  m_tCurXW  = m_tW - rtActualVal;
  m_tSumXW += m_tCurXW;

  // We might have to limit our values...
  if (m_bLimit) {
    return std::max(static_cast<float>(-m_tLimP),
                    std::min(m_fKp * static_cast<float>(m_tCurXW),
                             static_cast<float>(m_tLimP)))
         + std::max(static_cast<float>(-m_tLimI),
                    std::min(m_fKi * fTime * static_cast<float>(m_tSumXW),
                             static_cast<float>(m_tLimI)))
         + std::max(static_cast<float>(-m_tLimD),
                    std::min(m_fKd * static_cast<float>(m_tCurXW - m_tLastXW) / fTime,
                             static_cast<float>(m_tLimD)));
  }
  // do not have to limit our values.
  return m_fKp * m_tCurXW
       + m_fKi * fTime * m_tSumXW
       + m_fKd * (m_tCurXW - m_tLastXW) / fTime;
}


template<class T> inline void PID<T>::setLimits(const T& rtLimP,
                                                const T& rtLimI,
                                                const T& rtLimD)
{
  m_tLimP = rtLimP;
  m_tLimI = rtLimI;
  m_tLimD = rtLimD;
}


template<class T> inline void PID<T>::activateLimits(const bool bOn)
{
  m_bLimit = bOn;
}


template<class T> inline T PID<T>::getW() const
{
  return m_tW;
}


template<class T> inline void PID<T>::setW(const T& rtW)
{
  m_tSumXW = 0;
  m_tW     = rtW;
}



} // namespace attempto

#endif // PID_H_INCLUDED
