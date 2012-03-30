//------------------------------------------------------------------------------
// Graphing functions for OpenCV.	Part of "ImageUtils.cpp", a set of handy utility functions for dealing with images in OpenCV.
// by Shervin Emami (http://www.shervinemami.co.cc/) on 20th May, 2010.
//------------------------------------------------------------------------------

#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#ifdef __cplusplus
extern "C"
{
#endif

// Allow 'bool' variables in both C and C++ code.
#ifdef __cplusplus
#else
	typedef int bool;
	#define true (1)
	#define false (0)
#endif

#ifdef __cplusplus
    #define DEFAULT(val) = val
#else
    #define DEFAULT(val)
#endif

//------------------------------------------------------------------------------
// Graphing functions
//------------------------------------------------------------------------------

// Draw the graph of an array of floats into imageDst or a new image, between minV & maxV if given.
// Remember to free the newly created image if imageDst is not given.
IplImage* drawFloatGraph(const float *arraySrc, int nArrayLength, IplImage *imageDst DEFAULT(0), float minV DEFAULT(0.0), float maxV DEFAULT(0.0), int width DEFAULT(0), int height DEFAULT(0), char *graphLabel DEFAULT(0), bool showScale DEFAULT(true));

// Draw the graph of an array of ints into imageDst or a new image, between minV & maxV if given.
// Remember to free the newly created image if imageDst is not given.
IplImage* drawIntGraph(const int *arraySrc, int nArrayLength, IplImage *imageDst DEFAULT(0), int minV DEFAULT(0), int maxV DEFAULT(0), int width DEFAULT(0), int height DEFAULT(0), char *graphLabel DEFAULT(0), bool showScale DEFAULT(true));

// Draw the graph of an array of uchars into imageDst or a new image, between minV & maxV if given.
// Remember to free the newly created image if imageDst is not given.
IplImage* drawUCharGraph(const uchar *arraySrc, int nArrayLength, IplImage *imageDst DEFAULT(0), int minV DEFAULT(0), int maxV DEFAULT(0), int width DEFAULT(0), int height DEFAULT(0), char *graphLabel DEFAULT(0), bool showScale DEFAULT(true));

// Display a graph of the given float array.
// If background is provided, it will be drawn into, for combining multiple graphs using drawFloatGraph().
// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
void showFloatGraph(const char *name, const float *arraySrc, int nArrayLength, int delay_ms DEFAULT(500), IplImage *background DEFAULT(0));

// Display a graph of the given int array.
// If background is provided, it will be drawn into, for combining multiple graphs using drawIntGraph().
// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
void showIntGraph(const char *name, const int *arraySrc, int nArrayLength, int delay_ms DEFAULT(500), IplImage *background DEFAULT(0));

// Display a graph of the given unsigned char array.
// If background is provided, it will be drawn into, for combining multiple graphs using drawUCharGraph().
// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
void showUCharGraph(const char *name, const uchar *arraySrc, int nArrayLength, int delay_ms DEFAULT(500), IplImage *background DEFAULT(0));

// Simple helper function to easily view an image, with an optional pause.
void showImage(const IplImage *img, int delay_ms DEFAULT(0), char *name DEFAULT(0));

// Call 'setGraphColor(0)' to reset the colors that will be used for graphs.
void setGraphColor(int index DEFAULT(0));
// Specify the exact color that the next graph should be drawn as.
void setCustomGraphColor(int R, int B, int G);



#if defined (__cplusplus)
}
#endif

#endif //end GRAPH_UTILS