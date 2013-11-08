#include "runningstat.h"
#include <cmath>

RunningStat::RunningStat() : m_n(0)
{
}

void RunningStat::Clear()
{
    m_n = 0;
}

void RunningStat::Push(double x)
{
    m_n++;

    // See Knuth TAOCP vol 2, 3rd edition, page 232
    if (m_n == 1)
    {
        m_oldM = m_newM = x;
        m_oldS = 0.0;
    }
    else
    {
        m_newM = m_oldM + (x - m_oldM)/m_n;
        m_newS = m_oldS + (x - m_oldM)*(x - m_newM);

        // set up for next iteration
        m_oldM = m_newM; 
        m_oldS = m_newS;
    }
}

int RunningStat::NumDataValues() const
{
    return m_n;
}

double RunningStat::Mean() const
{
    return (m_n > 0) ? m_newM : 0.0;
}

double RunningStat::Variance() const
{
    return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
}

double RunningStat::StandardDeviation() const
{
    return std::sqrt( Variance() );
}
