// Incremental calculation of mean and variance
// From: http://www.johndcook.com/standard_deviation.html
class RunningStat
{
public:
    RunningStat();

    void Clear();
    void Push(double x);
    int NumDataValues() const;
    double Mean() const;
    double Variance() const;
    double StandardDeviation() const;

private:
    int m_n;
    double m_oldM, m_newM, m_oldS, m_newS;
};
