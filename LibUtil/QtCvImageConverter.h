#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace QtCvImageConverter {

/**
 * @brief The QTRGBConverter struct is the default rgb mapper
 */
struct QTRGBConverter {
static inline unsigned int rgb(int r, int g, int b) // set RGB value
{ return (0xffu << 24) | ((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff); }

static inline unsigned int rgba(int r, int g, int b, int a) // set RGBA value
{ return ((a & 0xff) << 24) | ((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff); }
};

/// FORWARD DECLARATION
class QImage;

/**
 * @brief The Converter class is a helper class template for converting QImages to cv images
 * @note It is a template so that Qt doesn't have to be linked to this library
 */
template <class TargetClass, template <class> class Pointer, class RGBConverter = QTRGBConverter>
class Converter
{
private:
    /**
     * @brief Tools
     */
    Converter();

public:
    /**
     * @brief mat2QImage converts an OpenCV image to a Qt image
     * @param mat OpenCV image
     * @return Qt image
     */
    static Pointer<TargetClass> mat2QImage(const cv::Mat &mat) {
        const IplImage& i = mat;
        return ipl2QImage(&i);
    }

    /**
     * @brief mat2QImage converts an OpenCV image to a Qt image
     * @param iplImg OpenCV
     * @return Qt image image
     */
    static Pointer<TargetClass> ipl2QImage(const IplImage *iplImg) {
        int h = iplImg->height;
        int w = iplImg->width;
        int channels = iplImg->nChannels;
        TargetClass* qimg = new TargetClass(w, h, TargetClass::Format_ARGB32);
        char* data = iplImg->imageData;

        assert(w > 0);
        assert(h > 0);
        assert(data != NULL);

        for (int y = 0; y < h; y++, data += iplImg->widthStep) {
            for (int x = 0; x < w; x++) {
                char r = 0, g = 0, b = 0, a = 0;
                if (channels == 1) {
                    r = data[x * channels];
                    g = data[x * channels];
                    b = data[x * channels];
                } else if (channels == 3 || channels == 4) {
                    r = data[x * channels + 2];
                    g = data[x * channels + 1];
                    b = data[x * channels];
                }

                if (channels == 4) {
                    a = data[x * channels + 3];
                    qimg->setPixel(x, y, RGBConverter::rgba(r, g, b, a));
                } else {
                    qimg->setPixel(x, y, RGBConverter::rgb(r, g, b));
                }
            }
        }
        return Pointer<TargetClass>(qimg);
    }
};

}

#endif // IMAGE_CONVERTER_H
