#ifndef UTILS_DIFF_NO_SIMD
#define UTILS_DIFF_NO_SIMD


/**
 * @brief Implementation of core functions without SIMD
 */
namespace Utils_DIFF
{

/**
 * @brief Calculate the minimum and position of the difference between height map and height image of a wheel
 */
static int diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, int &rxp, int &ryp)
{
    const short *srcP = (input.ptr<short>(ty)+tx);
    const short *curSrcP = srcP;
    const short *tmpP = (temp.ptr<short>(0));
    const short *tmpPEnd;
    const int numStepTemp = temp.step/2;
    const int numStepImg = input.step/2;
    //const int diffStep = numStepImg-numStepTemp;

    //int val = 30000;
    int tval = 30000;
    int x = 0;

    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const short a = *srcP;
            const short b = *tmpP;
            const short val = b-a;


            if (val < tval)
            {
                tval = val;
                ryp = y;
                rxp = x;
            }

            ++srcP;
            ++tmpP;
            ++x;

        }
        curSrcP += numStepImg;
        srcP = curSrcP;
        x=0;

    }

    return tval;


}


/**
 * @brief Calculate the wheel support on the height map
 */
static int calcWheelSupport(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, const int &wsThresh , const int &zval)
{
    const short *srcP = (input.ptr<short>(ty)+tx);
    const short *curSrcP = srcP;
    const short *tmpP = (temp.ptr<short>(0));
    const short *tmpPEnd;
    const int numStepTemp = temp.step/2;
    const int numStepImg = input.step/2;
    const short tValA = zval;
    const short cmpVal = (wsThresh);
    //const int diffStep = numStepImg-numStepTemp;

    int val = 30000;
    int tval = 30000;
    int x = 0;
    short supVal = 0;

    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const short a = *srcP;
            const short b = *tmpP;
            const short res = b-a;
            const short res2 = res-tValA;

            if (res2 < cmpVal) ++supVal;

            ++srcP;
            ++tmpP;
            ++x;

        }
        curSrcP += numStepImg;
        srcP = curSrcP;
        x=0;

    }

    return supVal;


}


/**
 * @brief Calculate the minimum value, position of the minimum and wheel support on the height map and the height image of a wheel
 */
static int ws_diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, const int &wsThresh , int &rxp, int &ryp, int &wsRes)
{
    const short *srcP = (input.ptr<short>(ty)+tx);
    const short *curSrcP = srcP;
    const short *tmpP = (temp.ptr<short>(0));
    const short *tmpPEnd;
    const int numStepTemp = temp.step/2;
    const int numStepImg = input.step/2;
    //const int diffStep = numStepImg-numStepTemp;

    int val = 30000;
    int tval = 30000;
    int x = 0;

    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const short a = *srcP;
            const short b = *tmpP;
            const short val = b-a;


            if (val < tval)
            {
                tval = val;
                ryp = y;
                rxp = x;
            }

            ++srcP;
            ++tmpP;
            ++x;

        }
        curSrcP += numStepImg;
        srcP = curSrcP;
        x=0;

    }

    const short tValA = tval;
    const short cmpVal = (wsThresh);
    int supVal;

    srcP = (input.ptr<short>(ty)+tx);
    curSrcP = srcP;
    tmpP = (temp.ptr<short>(0));
    x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const short a = *srcP;
            const short b = *tmpP;
            const short res = b-a;
            const short res2 = res-tValA;

            if (res2 < cmpVal) ++supVal;
            ++srcP;
            ++tmpP;
            ++x;

        }
        curSrcP += numStepImg;
        srcP = curSrcP;
        x=0;

    }


    wsRes = (supVal);


    return tval;


}



/**
 * @brief Calculate the minimum value of the difference between height map and height image of a wheel
 */
static int np_diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, int &rxp, int &ryp)
{
    const short *srcP = (input.ptr<short>(ty)+tx);
    const short *curSrcP = srcP;
    const short *tmpP = (temp.ptr<short>(0));
    const short *tmpPEnd;
    const int numStepTemp = temp.step/2;
    const int numStepImg = input.step/2;
    //const int diffStep = numStepImg-numStepTemp;

    int tval = 30000;
    int x = 0;

    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const short a = *srcP;
            const short b = *tmpP;
            const short val = b-a;


            if (val < tval) tval = val;
            ++srcP;
            ++tmpP;
            ++x;

        }
        curSrcP += numStepImg;
        srcP = curSrcP;
        x=0;

    }

    return tval;


}

/**
 * @brief Test for chassis collision with finding the contact position
 */
static int testChassis(const cv::Mat &input, const cv::Mat &temp, const float &sval, const float &dx, const float &dy , const int &tx, const int &ty, int &rxp, int &ryp)
{


    //const __m128 incrX = _mm_set1_ps(dx*4.0f);
    //const __m128 incrY = _mm_set1_ps(dy);
    float yStart = sval;
    float curVals;// = yStart;



    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    const short *srcP;
    const short *tmpP;

    int tval = 30000;
    int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        srcP = input.ptr<short>(ty+y)+tx;
        tmpP = temp.ptr<short>(y);
        curVals = yStart;

        for (x = 0; x < temp.cols;x++)
        {

            curVals += dx;
            const short tInc = (short)curVals;
            const short a = *(srcP+x);
            const short b = *(tmpP+x);
            const short mb = (b+tInc);
            const short val =(mb-a);

            if (val < tval)
            {
                tval = val;
                ryp = y;
                rxp = x;
            }

        }

        yStart += dy;
    }

    return tval;

}


/**
 * @brief Test for chassis collision without finding the contact position. It only determines if a collision happened.
 */
static int np_testChassis(const cv::Mat &input, const cv::Mat &temp, const float &sval, const float &dx, const float &dy , const int &tx, const int &ty, int &rxp, int &ryp)
{


    float yStart = sval;
    float curVals;// = yStart;



    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    const short *srcP;
    const short *tmpP;

    int tval = 30000;
    int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        srcP = input.ptr<short>(ty+y)+tx;
        tmpP = temp.ptr<short>(y);
        curVals = yStart;

        for (x = 0; x < temp.cols;x++)
        {

            curVals += dx;
            const short tInc = (short)curVals;
            const short a = *(srcP+x);
            const short b = *(tmpP+x);
            const short mb = (b+tInc);
            const short val =(mb-a);

            if (val < tval)
            {
                tval = val;

            }

        }

        yStart += dy;
    }

    return tval;


}


/**
 * @brief Warp chassis according to pose estimate
 */
static void warpChassis(const cv::Mat &temp, cv::Mat &result, const float &sval, const float &dx, const float &dy)
{
    float yStart = sval;
    float curVals;// = yStart;



    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    const short *srcP;
    short *resP;
    int x = 0;

    for (int y = 0; y < temp.rows;++y)
    {
        srcP = temp.ptr<short>(y);
        resP = result.ptr<short>(y);

        for (x = 0; x < temp.cols;x++)
        {

            curVals += dx;
            const short tInc = (short)curVals;
            const short a = *(srcP+x);
            const short mb = (a+tInc);
            *resP = mb;

        }

        yStart += dy;
    }


}


}

#endif // UTILS_DIFF_SSE

