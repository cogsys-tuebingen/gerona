#ifndef CV_ALIGNED_MAT_H
#define CV_ALIGNED_MAT_H

// Workaround for compilers without mm_malloc
// only include mm_alloc if SSE or AVX are available, otherwise its not required anyway.
#if (defined(__SSE4_2__) || defined(__AVX2__))
#include <mm_malloc.h>
#define aligned_free(p) (_mm_free(p))
#define aligned_malloc(a, b) (_mm_malloc(a,b))
#else
#include <stdlib.h>
#define aligned_free(p) (free(p))
#define aligned_malloc(a, b) (malloc(a))
#endif

#include <memory>
#include <opencv2/core/core.hpp>

#define CVAlignedMat_Alignment 32

/**
 * @brief In some cases the 32-byte alignment required for AVX processing does not work, this helper class wraps a cv::Mat and ensures 32-byte alignment
 */
class CVAlignedMat
{

public:
    typedef std::shared_ptr<CVAlignedMat > ptr;

    static CVAlignedMat::ptr Create(int width, int height, int cvType){ return std::make_shared< CVAlignedMat >(width,height,cvType) ; }
    static CVAlignedMat::ptr Create(cv::Size imgSize, int cvType){ return std::make_shared< CVAlignedMat >(imgSize,cvType) ; }
    static CVAlignedMat::ptr Create(cv::Mat input){ return std::make_shared< CVAlignedMat >(input) ; }

    CVAlignedMat(int width, int height, int cvType)
    {
        Allocate(width,height,cvType);
    }

    CVAlignedMat(cv::Size imgSize, int cvType)
    {
        Allocate(imgSize.width,imgSize.height,cvType);
    }

    CVAlignedMat(const cv::Mat &input)
    {
        Allocate(input.cols,input.rows,input.type());
        int ByteSize = GetPixelSizeForCVTypes(input.type());

        for (int y = 0; y < input.rows;y++)
        {
            memcpy(mat_.ptr(y),input.ptr(y),input.cols*ByteSize);
        }
    }

    void Allocate(int width, int height, int cvType)
    {
        int ByteSize = GetPixelSizeForCVTypes(cvType);
        int allocateWidth = width;
        if ((allocateWidth*ByteSize) % CVAlignedMat_Alignment != 0)
        {
            allocateWidth += (CVAlignedMat_Alignment - (width*ByteSize)% CVAlignedMat_Alignment)/ByteSize;
        }
        void* p =(void*) aligned_malloc (allocateWidth*height*ByteSize, CVAlignedMat_Alignment);
        memset(p,0,allocateWidth*height*ByteSize);
        cv::Mat resMat(cv::Size(width,height),cvType,p,allocateWidth*ByteSize);
        mat_ = resMat;
    }

    void SetZero()
    {
        memset(mat_.data,0,mat_.step*mat_.rows);
    }

    void CopyDataFrom(const cv::Mat &input)
    {
        memset(mat_.data,0,mat_.step*mat_.rows);
        int ByteSize = GetPixelSizeForCVTypes(input.type());

        for (int y = 0; y < input.rows;y++)
        {
            memcpy(mat_.ptr(y),input.ptr(y),input.cols*ByteSize);
        }
    }

    void CopyDataFrom(const void* input)
    {
        int ByteSize = GetPixelSizeForCVTypes(mat_.type());


        memcpy(mat_.ptr(),input,mat_.step*ByteSize*mat_.rows);

    }


    ~CVAlignedMat(){
        aligned_free((void*)mat_.data);
        //std::cout << "destroyMat!!" << std::endl;
    }
    cv::Mat mat_;

    int GetPixelSizeForCVTypes(int cvType)
    {
    switch(cvType){
    case CV_8U  :
        return 1;
        break; //optional
    case CV_8UC3  :
        return 3;
        break; //optional
    case CV_8UC4  :
        return 4;
        break; //optional

    case CV_16S  :
        return 2;
        break; //optional
    case CV_16SC3  :
        return 6;
        break; //optional
    case CV_16SC4  :
        return 8;
        break; //optional

    case CV_16U  :
        return 2;
        break; //optional
    case CV_16UC3  :
        return 6;
        break; //optional
    case CV_16UC4  :
        return 8;
        break; //optional


    case CV_32F  :
        return 4;
        break; //optional
    case CV_32FC3  :
        return 12;
        break; //optional
    case CV_32FC4  :
        return 16;
        break; //optional

    case CV_64F  :
        return 8;
        break; //optional
    case CV_64FC3  :
        return 24;
        break; //optional
    case CV_64FC4  :
        return 32;
        break; //optional
    }
    return 1;

    }

};




#endif //CV_ALIGNED_MAT_H
