#ifndef UTILS_DIFF_SSE
#define UTILS_DIFF_SSE


#include <nmmintrin.h>


/**
 * @brief SSE implementation of core functions
 */
namespace Utils_DIFF
{

static int diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, int &rxp, int &ryp)
{
    const __m128i *srcP = (__m128i*)(input.ptr<short>(ty)+tx);
    const __m128i *curSrcP = srcP;
    const __m128i *tmpP = (__m128i*)(temp.ptr<short>(0));
    const __m128i *tmpPEnd;
    const int numStepTemp = temp.step/16;
    const int numStepImg = input.step/16;
    //const int diffStep = numStepImg-numStepTemp;

    int val = 30000;
    int tval = 30000;
    int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const __m128i a = _mm_loadu_si128((srcP));
            const __m128i b = _mm_load_si128(tmpP);
            const __m128i res =_mm_sub_epi16(b,a);
            const __m128i res2 = _mm_minpos_epu16( res);
            val = _mm_extract_epi16(res2,0);


            if (val < tval)
            {
                tval = val;
                ryp = y;
                rxp = (x*8)+_mm_extract_epi16(res2,1);
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


static inline uint32_t _mm_sum_epu8(const __m128i v)
{
    __m128i vsum = _mm_sad_epu8(v, _mm_setzero_si128());
    return _mm_extract_epi16(vsum, 0) + _mm_extract_epi16(vsum, 4);
}

static int calcWheelSupport(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, const int &wsThresh , const int &zval)
{
    const __m128i *srcP = (__m128i*)(input.ptr<short>(ty)+tx);
    const __m128i *curSrcP = srcP;
    const __m128i *tmpP = (__m128i*)(temp.ptr<short>(0));
    const __m128i *tmpPEnd;
    const int numStepTemp = temp.step/16;
    const int numStepImg = input.step/16;
    //const int diffStep = numStepImg-numStepTemp;


    const __m128i tValA = _mm_set1_epi16(zval);
    const __m128i cmpVal = _mm_set1_epi16(wsThresh);
    const __m128i ones = _mm_set1_epi16(1);
    __m128i supVal = _mm_set1_epi16(0);
    int x = 0;
    srcP = (__m128i*)(input.ptr<short>(ty)+tx);
    curSrcP = srcP;
    tmpP = (__m128i*)(temp.ptr<short>(0));
    x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const __m128i a = _mm_loadu_si128((srcP));
            const __m128i b = _mm_load_si128(tmpP);
            const __m128i res =_mm_sub_epi16(b,a);
            const __m128i res2 = _mm_sub_epi16(res,tValA);
            const __m128i cmp = _mm_cmplt_epi16(res2,cmpVal);
            const __m128i sup = _mm_and_si128(ones,cmp);
            supVal = _mm_add_epi16(supVal,sup);
            ++srcP;
            ++tmpP;
            ++x;

        }
        curSrcP += numStepImg;
        srcP = curSrcP;
        x=0;

    }


    return _mm_sum_epu8(supVal);


}


static int ws_diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, const int &wsThresh , int &rxp, int &ryp, int &wsRes)
{
    const __m128i *srcP = (__m128i*)(input.ptr<short>(ty)+tx);
    const __m128i *curSrcP = srcP;
    const __m128i *tmpP = (__m128i*)(temp.ptr<short>(0));
    const __m128i *tmpPEnd;
    const int numStepTemp = temp.step/16;
    const int numStepImg = input.step/16;
    //const int diffStep = numStepImg-numStepTemp;

    int val = 30000;
    int tval = 30000;
    int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const __m128i a = _mm_loadu_si128((srcP));
            const __m128i b = _mm_load_si128(tmpP);
            const __m128i res =_mm_sub_epi16(b,a);
            const __m128i res2 = _mm_minpos_epu16( res);
            val = _mm_extract_epi16(res2,0);


            if (val < tval)
            {
                tval = val;
                ryp = y;
                rxp = (x*8)+_mm_extract_epi16(res2,1);
            }

            ++srcP;
            ++tmpP;
            ++x;

        }
        curSrcP += numStepImg;
        srcP = curSrcP;
        x=0;

    }

    const __m128i tValA = _mm_set1_epi16(tval);
    const __m128i cmpVal = _mm_set1_epi16(wsThresh);
    const __m128i ones = _mm_set1_epi16(1);
    __m128i supVal = _mm_set1_epi16(0);

    srcP = (__m128i*)(input.ptr<short>(ty)+tx);
    curSrcP = srcP;
    tmpP = (__m128i*)(temp.ptr<short>(0));
    x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const __m128i a = _mm_loadu_si128((srcP));
            const __m128i b = _mm_load_si128(tmpP);
            const __m128i res =_mm_sub_epi16(b,a);
            const __m128i res2 = _mm_sub_epi16(res,tValA);
            const __m128i cmp = _mm_cmplt_epi16(res2,cmpVal);
            const __m128i sup = _mm_and_si128(ones,cmp);
            supVal = _mm_add_epi16(supVal,sup);
            ++srcP;
            ++tmpP;
            ++x;

        }
        curSrcP += numStepImg;
        srcP = curSrcP;
        x=0;

    }


    wsRes = _mm_sum_epu8(supVal);


    return tval;


}

static int get_wheel_support(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, const int &zDiff, const int &wsThresh)
{

    const __m128i tValA = _mm_set1_epi16(zDiff);
    const __m128i cmpVal = _mm_set1_epi16(wsThresh);
    const __m128i ones = _mm_set1_epi16(1);
    __m128i supVal = _mm_set1_epi16(0);

    const __m128i* srcP = (__m128i*)(input.ptr<short>(ty)+tx);
    const __m128i* curSrcP = srcP;
    const __m128i* tmpP = (__m128i*)(temp.ptr<short>(0));
    const __m128i* tmpPEnd;
    const int numStepTemp = temp.step/16;
    const int numStepImg = input.step/16;


    for (int y = 0; y < temp.rows;++y)
    {
        tmpPEnd = tmpP+numStepTemp;
        while (tmpP != tmpPEnd)
        {
            const __m128i a = _mm_loadu_si128((srcP));
            const __m128i b = _mm_load_si128(tmpP);
            const __m128i res =_mm_sub_epi16(b,a);
            const __m128i res2 = _mm_sub_epi16(res,tValA);
            const __m128i cmp = _mm_cmplt_epi16(res2,cmpVal);
            const __m128i sup = _mm_and_si128(ones,cmp);
            supVal = _mm_add_epi16(supVal,sup);
            ++srcP;
            ++tmpP;

        }
        curSrcP += numStepImg;
        srcP = curSrcP;


    }

    return _mm_sum_epu8(supVal);


}


static int np_diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, int &rxp, int &ryp)
{
    const __m128i *srcP = (__m128i*)(input.ptr<short>(ty)+tx);;
    const __m128i *tmpP = (__m128i*)(temp.ptr<short>(0));
    const __m128i *endTmpP;

    const int numStepsTemp = temp.step/16;
    const int numStepsImg = input.step/16;
    const int diffStep = numStepsImg-numStepsTemp;


    __m128i bestVals = _mm_set1_epi16(30000);


    //const int numSteps = temp.step/16;

    for (int y = 0; y < temp.rows;++y)
    {

        endTmpP = tmpP+numStepsTemp;
        while (tmpP != endTmpP)
        {
            const __m128i a = _mm_loadu_si128((srcP));
            const __m128i b = _mm_load_si128((tmpP));
            const __m128i res =_mm_sub_epi16(b,a);

            bestVals = _mm_min_epi16(res,bestVals);

            ++tmpP;
            ++srcP;

        }
        srcP+=diffStep;



    }
    const __m128i res2 = _mm_minpos_epu16( bestVals);
    int resVal = _mm_extract_epi16(res2,0);

    return resVal;


}


static int testChassis(const cv::Mat &input, const cv::Mat &temp, const float &sval, const float &dx, const float &dy , const int &tx, const int &ty, int &rxp, int &ryp)
{


    const __m128 incrX = _mm_set1_ps(dx*4.0f);
    const __m128 incrY = _mm_set1_ps(dy);
    __m128 yStart = _mm_set_ps(sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval);
    __m128 curVals;// = yStart;



    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    const short *srcP;
    const short *tmpP;

    int val = 30000;
    int tval = 30000;
    int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        srcP = input.ptr<short>(ty+y)+tx;
        tmpP = temp.ptr<short>(y);
        curVals = yStart;

        for (x = 0; x < temp.cols;x+=8)
        {

            const __m128 c1 = _mm_add_ps(curVals,incrX);

            const __m128i c1i = _mm_cvtps_epi32(curVals);
            const __m128i c2i = _mm_cvtps_epi32(c1);
            const __m128i tInc = _mm_packs_epi32(c1i,c2i);

            curVals = _mm_add_ps(c1,incrX);

            const __m128i a = _mm_loadu_si128((const __m128i*)(srcP+x));
            const __m128i b = _mm_load_si128((const __m128i*)(tmpP+x));
            const __m128i mb = _mm_add_epi16(b,tInc);
            const __m128i res =_mm_sub_epi16(mb,a);
            const __m128i res2 = _mm_minpos_epu16( res);

            val = _mm_extract_epi16(res2,0);


            if (val < tval)
            {
                tval = val;
                ryp = y;
                rxp = x+_mm_extract_epi16(res2,1);
            }

        }

        yStart = _mm_add_ps(yStart,incrY);
    }

    return tval;

}


static int np_testChassis(const cv::Mat &input, const cv::Mat &temp, const float &sval, const float &dx, const float &dy , const int &tx, const int &ty, int &rxp, int &ryp)
{


    const __m128 incrX = _mm_set1_ps(dx*4.0f);
    const __m128 incrY = _mm_set1_ps(dy);
    __m128 yStart = _mm_set_ps(sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval);
    __m128 curVals;// = yStart;


    const int numStepTemp = temp.step/16;
    const int numStepImg = input.step/16;
    const int diffStep = numStepImg-numStepTemp;


    __m128i bestVals = _mm_set1_epi16(30000);


    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    const __m128i *srcP = (__m128i*)(input.ptr<short>(ty)+tx);
    const __m128i *tmpP = (__m128i*)(temp.ptr<short>(0));
    const __m128i *endTmpP;
    //int val = 30000;
    //int tval = 30000;
    //int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {

        endTmpP = tmpP+numStepTemp;

        curVals = yStart;

        while (tmpP != endTmpP)
        {

            const __m128 c1 = _mm_add_ps(curVals,incrX);

            const __m128i c1i = _mm_cvtps_epi32(curVals);
            const __m128i c2i = _mm_cvtps_epi32(c1);
            const __m128i tInc = _mm_packs_epi32(c1i,c2i);

            curVals = _mm_add_ps(c1,incrX);

            const __m128i a = _mm_loadu_si128((srcP));
            const __m128i b = _mm_load_si128((tmpP));
            const __m128i mb = _mm_add_epi16(b,tInc);
            const __m128i res =_mm_sub_epi16(mb,a);

            bestVals = _mm_min_epi16(res,bestVals);


            ++srcP;
            ++tmpP;


        }

        srcP += diffStep;
        yStart = _mm_add_ps(yStart,incrY);
    }

    const __m128i res2 = _mm_minpos_epu16( bestVals);
    return _mm_extract_epi16(res2,0);


}


static void warpChassis(const cv::Mat &temp, cv::Mat &result, const float &sval, const float &dx, const float &dy)
{


    const __m128 incrX = _mm_set1_ps(dx*4.0f);
    const __m128 incrY = _mm_set1_ps(dy);
    __m128 yStart = _mm_set_ps(sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval);
    //__m128 yStart = _mm_set_ps(sval,sval+dx,sval+dx*2.0f,sval+dx*3.0f);
    __m128 curVals;// = yStart;



    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    const short *srcP;
    short *resP;


    int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        srcP = temp.ptr<short>(y);
        resP = result.ptr<short>(y);
        curVals = yStart;

        for (x = 0; x < temp.cols;x+=8)
        {

            const __m128 c1 = _mm_add_ps(curVals,incrX);

            const __m128i c1i = _mm_cvtps_epi32(curVals);
            const __m128i c2i = _mm_cvtps_epi32(c1);
            const __m128i tInc = _mm_packs_epi32(c1i,c2i);

            curVals = _mm_add_ps(c1,incrX);


            const __m128i b = _mm_load_si128((const __m128i*)(srcP+x));
            const __m128i mb = _mm_add_epi16(b,tInc);

            _mm_storeu_si128((__m128i*)(resP+x),mb);


        }

        yStart = _mm_add_ps(yStart,incrY);
    }

    return;

}


}

#endif // UTILS_DIFF_SSE

