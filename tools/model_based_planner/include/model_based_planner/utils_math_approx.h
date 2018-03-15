#ifndef UTILS_MATH_APPROX_H
#define UTILS_MATH_APPROX_H


/**
 * @brief Approximate math functions (faster)
 */


//#define USE_ACOS_APPROX 1
#define USE_RSQRT_APPROX 1
#define USE_ATAN2_APPROX 1


#define CV_PIF   3.1415926535897932384626433832795f
#define CV_2PIF 6.283185307179586476925286766559f

namespace Utils_Math_Approx
{
#ifdef USE_ACOS_APPROX

    static inline float facos(float x) {
      float ret = -0.0187293f;
      ret = ret * x;
      ret = ret + 0.0742610f;
      ret = ret * x;
      ret = ret - 0.2121144f;
      ret = ret * x;
      ret = ret + 1.5707288f;
      ret = ret * sqrt(1.0f-x);
      //ret = ret - 2 * negate * ret;
      return ret;
    }

    static inline float facos2(float x) {
      float negate = float(x < 0);
      x = std::abs(x);
      float ret = -0.0187293;
      ret = ret * x;
      ret = ret + 0.0742610;
      ret = ret * x;
      ret = ret - 0.2121144;
      ret = ret * x;
      ret = ret + 1.5707288;
      ret = ret * sqrt(1.0-x);
      ret = ret - 2 * negate * ret;
      return negate * 3.14159265358979 + ret;
    }
#else
static inline float facos(const float x) {
  return acos(x);
}
#endif

#ifdef USE_RSQRT_APPROX


    inline float frsqrt(const float number )
    {
        int i;
        float x2, y;
        const float threehalfs = 1.5F;

        x2 = number * 0.5F;
        y  = number;
        i  = * ( int * ) &y;                       // evil floating point bit level hacking
        i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
        y  = * ( float * ) &i;
        y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
        y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

        return y;
    }

#else
inline float frsqrt(float x) {
  return 1.0f/sqrt(x);
}
#endif

#ifdef USE_ATAN2_APPROX

static inline float fatan2(const float y, const float x)
{
    // https://codereview.stackexchange.com/questions/174617/avx-assembly-for-fast-atan2-approximation
    using namespace std;

    if (x == 0 && y== 0) {
        return 0;
    }

    // 7th order polynomial approximation of atan(z) on [-1,1], slightly
    // tweaked to remove a multiply at the cost of very slightly higher
    // error.
    const float a = min(std::abs(x),std::abs(y))/max(std::abs(x),std::abs(y));
    const float s = a*a;
    float r = ((-0.0464964749f*s + 0.15931422f)*s - 0.327622764f)*s*a + a;

    if (std::abs(y) > std::abs(x)) r = (float)M_PI_2 - r;
    if (x < 0)           r = (float)M_PI   - r;
    if (y < 0)           r =               - r;

    return r;
}

static inline float fatan2b(const float y, const float x)
{
    //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    //Volkan SALMA

    const float ONEQTR_PI = M_PI / 4.0;
    const float THRQTR_PI = 3.0 * M_PI / 4.0;
    float r, angle;
    float abs_y = std::abs(y) + 1e-10f;      // kludge to prevent 0/0 condition
    if ( x < 0.0f )
    {
        r = (x + abs_y) / (abs_y - x);
        angle = THRQTR_PI;
    }
    else
    {
        r = (x - abs_y) / (x + abs_y);
        angle = ONEQTR_PI;
    }
    angle += (0.1963f * r * r - 0.9817f) * r;
    if ( y < 0.0f )
        return( -angle );     // negate if in quad III or IV
    else
        return( angle );


}


#else
inline float fatan2(const float y, const float x) {
  return atan2(y,x);
}
#endif

}


#endif // UTILS_DIFF_H
