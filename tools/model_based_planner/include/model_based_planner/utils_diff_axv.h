#ifndef UTILS_DIFF_AXV
#define UTILS_DIFF_AXV


#include <immintrin.h>


/**
 * @brief AVX implementation of core functions
 */
namespace Utils_DIFF
{

/**
 * @brief Get index of smalles value of array v
 */
inline static int n_mm256_hmin_index(const __m256i &v, int &val)
{
    __m256i vmax = v;

    vmax = _mm256_min_epi16(vmax, _mm256_alignr_epi8(vmax, vmax, 2));
    vmax = _mm256_min_epi16(vmax, _mm256_alignr_epi8(vmax, vmax, 4));
    vmax = _mm256_min_epi16(vmax, _mm256_alignr_epi8(vmax, vmax, 8));
    vmax = _mm256_min_epi16(vmax, _mm256_permute2x128_si256(vmax, vmax, 0x01));

    const __m256i vcmp = _mm256_cmpeq_epi16(v, vmax);

    uint32_t mask = _mm256_movemask_epi8(vcmp);


    val = _mm256_extract_epi16(vmax,0);
    return  __builtin_ctz(mask) / 2;
}

/**
 * @brief Get smallest value of v
 */
inline static int n_mm256_hmin_val(const __m256i &v)
{
    __m256i vmax = v;

    vmax = _mm256_min_epi16(vmax, _mm256_alignr_epi8(vmax, vmax, 2));
    vmax = _mm256_min_epi16(vmax, _mm256_alignr_epi8(vmax, vmax, 4));
    vmax = _mm256_min_epi16(vmax, _mm256_alignr_epi8(vmax, vmax, 8));
    vmax = _mm256_min_epi16(vmax, _mm256_permute2x128_si256(vmax, vmax, 0x01));

    return _mm256_extract_epi16(vmax,0);

}


/**
 * @brief Calculate the minimum and position of the difference between height map and height image of a wheel
 */
static int diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, int &rxp, int &ryp)
{
    //int x = 0;
    const __m256i* msrcPtr = (const __m256i*)(input.ptr<short>(ty)+tx);
    const __m256i* mtempPtr= (const __m256i*)(temp.ptr<short>(0));
    const __m256i* endTempPtr;
    const int numStepsTemp = temp.step/32;
    const int numStepsImg = input.step/32;
    const int diffStep = numStepsImg-numStepsTemp;

    __m256i bestValues = _mm256_set1_epi16(30000);
    const __m256i inc = _mm256_set1_epi16(16);
    __m256i curIdx = _mm256_setr_epi16(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15);


    __m256i bestIdx = _mm256_set1_epi16(0);

    for (int y = 0; y < temp.rows;++y)
    {
        endTempPtr = mtempPtr+numStepsTemp;

        while (mtempPtr != endTempPtr)
        {
            const __m256i a = _mm256_loadu_si256((msrcPtr));
            const __m256i b = _mm256_load_si256((mtempPtr));
            const __m256i res =_mm256_sub_epi16(b,a);

            //bestValues = _mm256_min_epi16(bestValues,res);

            const __m256i cmp = _mm256_cmpgt_epi16(res,bestValues);
            const __m256i p1 = _mm256_and_si256(cmp,bestValues);
            const __m256i p2 = _mm256_andnot_si256(cmp,res);
            bestValues = _mm256_add_epi16(p1,p2);
            const __m256i i1 = _mm256_and_si256(cmp,bestIdx);
            const __m256i i2 = _mm256_andnot_si256(cmp,curIdx);
            bestIdx = _mm256_add_epi16(i1,i2);
            curIdx = _mm256_add_epi16(curIdx,inc);

            ++msrcPtr;
            ++mtempPtr;

        }
        msrcPtr += diffStep;

    }

    int zval;
    const int idx = n_mm256_hmin_index(bestValues,zval);

    short arr[16];
    _mm256_storeu_si256((__m256i*)arr, bestIdx);

    const short resPos = arr[idx];

    //const short resPos = ((short *)&bestIdx)[idx];
    rxp = resPos%temp.cols;
    ryp = resPos/temp.cols;

    return zval;


}

/**
 * @brief Calculate the minimum value of the difference between height map and height image of a wheel
 */
static int np_diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, int &rxp, int &ryp)
{
    //int x = 0;
    const __m256i* msrcPtr = (const __m256i*)(input.ptr<short>(ty)+tx);
    const __m256i* mtempPtr= (const __m256i*)(temp.ptr<short>(0));
    const __m256i* endTempPtr;
    const int numStepsTemp = temp.step/32;
    const int numStepsImg = input.step/32;
    const int diffStep = numStepsImg-numStepsTemp;

    __m256i bestValues = _mm256_set1_epi16(30000);

    for (int y = 0; y < temp.rows;++y)
    {
        endTempPtr = mtempPtr+numStepsTemp;

        while (mtempPtr != endTempPtr)
        {
            const __m256i a = _mm256_loadu_si256((msrcPtr));
            const __m256i b = _mm256_load_si256((mtempPtr));
            const __m256i res =_mm256_sub_epi16(b,a);

            bestValues = _mm256_min_epi16(bestValues,res);

            ++msrcPtr;
            ++mtempPtr;

        }
        msrcPtr += diffStep;

    }

    return n_mm256_hmin_val(bestValues);

}


/**
 * @brief Horizontal sum of val
 */
inline int HSumAvxI(const __m256i &val)
    {
        short tres[16];
        _mm256_storeu_si256((__m256i*) tres, val );

        return tres[0]+tres[1]+tres[2]+tres[3]+tres[4]+tres[5]+tres[6]+tres[7]+tres[8]+tres[9]+tres[10]+tres[11]+tres[12]+tres[13]+tres[14]+tres[15];

    }

/**
 * @brief Calculate the minimum value and wheel support of the difference between height map and height image of a wheel
 */
static int wsnp_diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, const int &wsThresh, int &wsRes)
{
    //int x = 0;
    const __m256i* msrcPtr = (const __m256i*)(input.ptr<short>(ty)+tx);
    const __m256i* mtempPtr= (const __m256i*)(temp.ptr<short>(0));
    const __m256i* endTempPtr;
    const int numStepsTemp = temp.step/32;
    const int numStepsImg = input.step/32;
    const int diffStep = numStepsImg-numStepsTemp;

    __m256i bestValues = _mm256_set1_epi16(30000);

    for (int y = 0; y < temp.rows;++y)
    {
        endTempPtr = mtempPtr+numStepsTemp;

        while (mtempPtr != endTempPtr)
        {
            const __m256i a = _mm256_loadu_si256((msrcPtr));
            const __m256i b = _mm256_load_si256((mtempPtr));
            const __m256i res =_mm256_sub_epi16(b,a);

            bestValues = _mm256_min_epi16(bestValues,res);

            ++msrcPtr;
            ++mtempPtr;

        }
        msrcPtr += diffStep;

    }

    const int zval = n_mm256_hmin_val(bestValues);

    msrcPtr = (const __m256i*)(input.ptr<short>(ty)+tx);
    mtempPtr= (const __m256i*)(temp.ptr<short>(0));

    const __m256i tValA = _mm256_set1_epi16(zval);
    const __m256i cmpVal = _mm256_set1_epi16(wsThresh);
    const __m256i ones = _mm256_set1_epi16(1);
    __m256i supVal = _mm256_set1_epi16(0);


    for (int y = 0; y < temp.rows;++y)
    {
        endTempPtr = mtempPtr+numStepsTemp;

        while (mtempPtr != endTempPtr)
        {
            const __m256i a = _mm256_loadu_si256((msrcPtr));
            const __m256i b = _mm256_load_si256((mtempPtr));
            const __m256i res =_mm256_sub_epi16(b,a);
            const __m256i res2 = _mm256_sub_epi16(res,tValA);
            const __m256i cmp = _mm256_cmpgt_epi16(cmpVal,res2);
            const __m256i sup = _mm256_and_si256(ones,cmp);
            supVal = _mm256_add_epi16(supVal,sup);
            ++msrcPtr;
            ++mtempPtr;

        }
        msrcPtr += diffStep;

    }


    wsRes = HSumAvxI(supVal);

    return zval;

}




/**
 * @brief Calculate the wheel support on the height map
 */
static int calcWheelSupport(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, const int &wsThresh , const int &zval)
{
    //int x = 0;
    const __m256i* msrcPtr = (const __m256i*)(input.ptr<short>(ty)+tx);
    const __m256i* mtempPtr= (const __m256i*)(temp.ptr<short>(0));
    const __m256i* endTempPtr;
    const int numStepsTemp = temp.step/32;
    const int numStepsImg = input.step/32;
    const int diffStep = numStepsImg-numStepsTemp;

    msrcPtr = (const __m256i*)(input.ptr<short>(ty)+tx);
    mtempPtr= (const __m256i*)(temp.ptr<short>(0));

    const __m256i tValA = _mm256_set1_epi16(zval);
    const __m256i cmpVal = _mm256_set1_epi16(wsThresh);
    const __m256i ones = _mm256_set1_epi16(1);
    __m256i supVal = _mm256_set1_epi16(0);


    for (int y = 0; y < temp.rows;++y)
    {
        endTempPtr = mtempPtr+numStepsTemp;

        while (mtempPtr != endTempPtr)
        {
            const __m256i a = _mm256_loadu_si256((msrcPtr));
            const __m256i b = _mm256_load_si256((mtempPtr));
            const __m256i res =_mm256_sub_epi16(b,a);
            const __m256i res2 = _mm256_sub_epi16(res,tValA);
            const __m256i cmp = _mm256_cmpgt_epi16(cmpVal,res2);
            const __m256i sup = _mm256_and_si256(ones,cmp);
            supVal = _mm256_add_epi16(supVal,sup);
            ++msrcPtr;
            ++mtempPtr;

        }
        msrcPtr += diffStep;

    }


    //wsRes = HSumAvxI(supVal);


    return HSumAvxI(supVal);


}

/**
 * @brief Calculate the minimum value, position of the minimum and wheel support on the height map and the height image of a wheel
 */
static int ws_diffMinPos(const cv::Mat &input, const cv::Mat &temp, const int &tx, const int &ty, const int &wsThresh , int &rxp, int &ryp, int &wsRes)
{
    //int x = 0;
    const __m256i* msrcPtr = (const __m256i*)(input.ptr<short>(ty)+tx);
    const __m256i* mtempPtr= (const __m256i*)(temp.ptr<short>(0));
    const __m256i* endTempPtr;
    const int numStepsTemp = temp.step/32;
    const int numStepsImg = input.step/32;
    const int diffStep = numStepsImg-numStepsTemp;

    __m256i bestValues = _mm256_set1_epi16(30000);
    const __m256i inc = _mm256_set1_epi16(16);
    __m256i curIdx = _mm256_setr_epi16(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15);


    __m256i bestIdx = _mm256_set1_epi16(0);

    for (int y = 0; y < temp.rows;++y)
    {
        endTempPtr = mtempPtr+numStepsTemp;

        while (mtempPtr != endTempPtr)
        {
            const __m256i a = _mm256_loadu_si256((msrcPtr));
            const __m256i b = _mm256_load_si256((mtempPtr));
            const __m256i res =_mm256_sub_epi16(b,a);

            //bestValues = _mm256_min_epi16(bestValues,res);

            const __m256i cmp = _mm256_cmpgt_epi16(res,bestValues);
            const __m256i p1 = _mm256_and_si256(cmp,bestValues);
            const __m256i p2 = _mm256_andnot_si256(cmp,res);
            bestValues = _mm256_add_epi16(p1,p2);
            const __m256i i1 = _mm256_and_si256(cmp,bestIdx);
            const __m256i i2 = _mm256_andnot_si256(cmp,curIdx);
            bestIdx = _mm256_add_epi16(i1,i2);
            curIdx = _mm256_add_epi16(curIdx,inc);

            ++msrcPtr;
            ++mtempPtr;

        }
        msrcPtr += diffStep;

    }

    int zval;
    const int idx = n_mm256_hmin_index(bestValues,zval);

    short arr[16];
    _mm256_storeu_si256((__m256i*)arr, bestIdx);

    const short resPos = arr[idx];

    //const short resPos = ((short *)&bestIdx)[idx];
    rxp = resPos%temp.cols;
    ryp = resPos/temp.cols;


    msrcPtr = (const __m256i*)(input.ptr<short>(ty)+tx);
    mtempPtr= (const __m256i*)(temp.ptr<short>(0));

    const __m256i tValA = _mm256_set1_epi16(zval);
    const __m256i cmpVal = _mm256_set1_epi16(wsThresh);
    const __m256i ones = _mm256_set1_epi16(1);
    __m256i supVal = _mm256_set1_epi16(0);


    for (int y = 0; y < temp.rows;++y)
    {
        endTempPtr = mtempPtr+numStepsTemp;

        while (mtempPtr != endTempPtr)
        {
            const __m256i a = _mm256_loadu_si256((msrcPtr));
            const __m256i b = _mm256_load_si256((mtempPtr));
            const __m256i res =_mm256_sub_epi16(b,a);
            const __m256i res2 = _mm256_sub_epi16(res,tValA);
            const __m256i cmp = _mm256_cmpgt_epi16(cmpVal,res2);
            const __m256i sup = _mm256_and_si256(ones,cmp);
            supVal = _mm256_add_epi16(supVal,sup);
            ++msrcPtr;
            ++mtempPtr;

        }
        msrcPtr += diffStep;

    }


    wsRes = HSumAvxI(supVal);


    return zval;


}


/**
 * @brief Test for chassis collision with finding the contact position
 */
static int testChassis(const cv::Mat &input, const cv::Mat &temp, const float &sval, const float &dx, const float &dy , const int &tx, const int &ty, int &rxp, int &ryp)
{

    int val = 30000;
    int tval = 30000;


    const __m256 incrX = _mm256_set1_ps(dx*4.0f);
    const __m256 incrX2 = _mm256_set1_ps(dx*12.0f);
    const __m256 incrY = _mm256_set1_ps(dy);
    //__m256 yStart = _mm256_set_ps(sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval,sval+dx*7.0f,sval+dx*6.0f,sval+dx*5.0f,sval+dx*4.0f);
    //__m256 yStart = _mm256_set_ps(sval+dx*7.0f,sval+dx*6.0f,sval+dx*5.0f,sval+dx*4.0f,sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval);
    __m256 yStart = _mm256_set_ps(sval+dx*11.0f,sval+dx*10.0f,sval+dx*9.0f,sval+dx*8.0f,sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval);
    __m256 curVals;// = yStart;


    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    const short *srcP;
    const short *tmpP;


    int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        srcP = input.ptr<short>(ty+y)+tx;
        tmpP = temp.ptr<short>(y);
        curVals = yStart;

        for (x = 0; x < temp.cols;x+=16)
        {

            const __m256 c1 = _mm256_add_ps(curVals,incrX);

            const __m256i c1i = _mm256_cvtps_epi32(curVals);
            const __m256i c2i = _mm256_cvtps_epi32(c1);
            const __m256i tInc = _mm256_packs_epi32(c1i,c2i);


            curVals = _mm256_add_ps(c1,incrX2);


            const __m256i a = _mm256_loadu_si256((const __m256i*)(srcP+x));
            const __m256i b = _mm256_load_si256((const __m256i*)(tmpP+x));
            const __m256i mb = _mm256_add_epi16(b,tInc);
            const __m256i res =_mm256_sub_epi16(mb,a);
            const __m128i lower = _mm256_extracti128_si256(res,0);
            const __m128i upper = _mm256_extracti128_si256(res,1);
            const __m128i resl = _mm_minpos_epu16( lower);
            const __m128i resu = _mm_minpos_epu16( upper);

            val = _mm_extract_epi16(resl,0);

            if (val < tval)
            {
                tval = val;
                ryp = y;
                rxp = x+_mm_extract_epi16(resl,1);
            }

            val = _mm_extract_epi16(resu,0);


            if (val < tval)
            {
                tval = val;
                ryp = y;
                rxp = x+8+_mm_extract_epi16(resu,1);
            }

        }

        yStart = _mm256_add_ps(yStart,incrY);
    }

    return tval;

}

/**
 * @brief Test for chassis collision without finding the contact position. It only determines if a collision happened.
 */
static int np_testChassis(const cv::Mat &input, const cv::Mat &temp, const float &sval, const float &dx, const float &dy , const int &tx, const int &ty, int &rxp, int &ryp)
{


    const __m256 incrX = _mm256_set1_ps(dx*4.0f);
    const __m256 incrX2 = _mm256_set1_ps(dx*12.0f);
    const __m256 incrY = _mm256_set1_ps(dy);
    __m256 yStart = _mm256_set_ps(sval+dx*11.0f,sval+dx*10.0f,sval+dx*9.0f,sval+dx*8.0f,sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval);
    __m256 curVals;// = yStart;


    __m256i resVals = _mm256_set1_epi16(30000);

    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    int x = 0;
    const __m256i* msrcPtr;
    const __m256i* mtempPtr;
    const int numSteps = temp.step/32;


    for (int y = 0; y < temp.rows;++y)
    {

        msrcPtr = (const __m256i*)(input.ptr<short>(ty+y)+tx);
        mtempPtr = (const __m256i*)(temp.ptr<short>(y));
        curVals = yStart;


        for (x = 0; x < numSteps;++x)
        {
            const __m256 c1 = _mm256_add_ps(curVals,incrX);

            const __m256i c1i = _mm256_cvtps_epi32(curVals);
            const __m256i c2i = _mm256_cvtps_epi32(c1);
            const __m256i tInc = _mm256_packs_epi32(c1i,c2i);


            curVals = _mm256_add_ps(c1,incrX2);


            const __m256i a = _mm256_loadu_si256(msrcPtr+x);
            const __m256i b = _mm256_load_si256(mtempPtr+x);


            const __m256i mb = _mm256_add_epi16(b,tInc);
            const __m256i res =_mm256_sub_epi16(mb,a);
            resVals = _mm256_min_epi16(resVals,res);



        }

        yStart = _mm256_add_ps(yStart,incrY);
    }



    return n_mm256_hmin_val(resVals);

}



/**
 * @brief Warp chassis according to pose estimate
 */
static void warpChassis(const cv::Mat &temp, cv::Mat &result, const float &sval, const float &dx, const float &dy)
{


    const __m256 incrX = _mm256_set1_ps(dx*4.0f);
    const __m256 incrX2 = _mm256_set1_ps(dx*12.0f);
    const __m256 incrY = _mm256_set1_ps(dy);
    __m256 yStart = _mm256_set_ps(sval+dx*11.0f,sval+dx*10.0f,sval+dx*9.0f,sval+dx*8.0f,sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval);
    //__m256 yStart = _mm256_set_ps(sval+dx*4.0f,sval+dx*5.0f,sval+dx*6.0f,sval+dx*7.0f,sval+dx*0.0f,sval+dx*1.0f,sval+dx*2.0f,sval+dx*3.0f);
    //__m256 yStart = _mm256_set_ps(sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval,sval+dx*7.0f,sval+dx*6.0f,sval+dx*5.0f,sval+dx*4.0f);
    //__m256 yStart = _mm256_set_ps(sval+dx*7.0f,sval+dx*6.0f,sval+dx*5.0f,sval+dx*4.0f,sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval);
    //__m256 yStart = _mm256_set_ps(sval+dx*0.0f,sval+dx*1.0f,sval+dx*2.0f,sval+dx*3.0f,sval+dx*4.0f,sval+dx*5.0f,sval+dx*6.0f,sval+dx*7.0f);
    __m256 curVals;// = yStart;


    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    const short *srcP;
    short *resP;


    int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        srcP = temp.ptr<short>(y);
        resP = result.ptr<short>(y);
        curVals = yStart;

        for (x = 0; x < temp.cols;x+=16)
        {

            const __m256 c1 = _mm256_add_ps(curVals,incrX);

            const __m256i c1i = _mm256_cvtps_epi32(curVals);
            const __m256i c2i = _mm256_cvtps_epi32(c1);
            const __m256i tInc = _mm256_packs_epi32(c1i,c2i);


            curVals = _mm256_add_ps(c1,incrX2);


            const __m256i b = _mm256_load_si256((__m256i*)(srcP+x));


            const __m256i mb = _mm256_add_epi16(b,tInc);
            _mm256_store_si256((__m256i*)(resP+x),mb);

        }

        yStart = _mm256_add_ps(yStart,incrY);
    }

    return;

}



static void warpChassis2(const cv::Mat &temp, cv::Mat &result, const float &sval, const float &dx, const float &dy)
{


    const __m256 incrX = _mm256_set1_ps(dx*8.0f);
    const __m256 incrY = _mm256_set1_ps(dy);
    //__m256 yStart = _mm256_set_ps(sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval,sval+dx*7.0f,sval+dx*6.0f,sval+dx*5.0f,sval+dx*4.0f);
    __m256 yStart = _mm256_set_ps(sval+dx*7.0f,sval+dx*6.0f,sval+dx*5.0f,sval+dx*4.0f,sval+dx*3.0f,sval+dx*2.0f,sval+dx,sval);
    __m256 curVals;// = yStart;


    /// 32 to 16 __m128i _mm_packus_epi32 (__m128i a, __m128i b)


    const short *srcP;
    short *resP;


    int x = 0;
    for (int y = 0; y < temp.rows;++y)
    {
        srcP = temp.ptr<short>(y);
        resP = result.ptr<short>(y);
        curVals = yStart;

        for (x = 0; x < temp.cols;x+=16)
        {

            const __m256 c1 = _mm256_add_ps(curVals,incrX);

            const __m256i c1i = _mm256_cvtps_epi32(curVals);
            const __m256i c2i = _mm256_cvtps_epi32(c1);
            const __m256i tInc = _mm256_packs_epi32(c1i,c2i);

            const __m256i tInc2 = _mm256_permute4x64_epi64(tInc,0xd8);

            curVals = _mm256_add_ps(c1,incrX);

            const __m256i a = _mm256_load_si256((const __m256i*)(srcP+x));
            const __m256i mb = _mm256_add_epi16(a,tInc2);
            _mm256_store_si256((__m256i*)(resP+x),mb);

        }

        yStart = _mm256_add_ps(yStart,incrY);
    }

    return;

}


}




#endif // UTILS_DIFF_AXV

