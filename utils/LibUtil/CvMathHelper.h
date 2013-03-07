/*
 * CvMathHelper.h
 *
 *  Created on: 27.09.2010
 *      Author: dube
 */

#ifndef CVMATHHELPER_H_
#define CVMATHHELPER_H_

#include <stddef.h>
#include <opencv/cv.h>

class CvMathHelper {
public:
	CvMathHelper();
	virtual ~CvMathHelper();

	static cv::Mat CreateRotationMatrix(double angleX, double angleY, double angleZ);
	static cv::Mat CreateTranslationMatrix(double deltaX, double deltaY, double deltaZ);
	static void PrintMatrix(const cv::Mat& matrix);
	static cv::Mat CreateVector(double x, double y, double z);
	static void printCvPoint2D32f(CvPoint2D32f point);
	static CvMat* generateRotationMatrix2D(float angle, CvPoint2D64f center);
	static CvMat* generateRotationMatrix2D(float angle);
	static CvMat* generateScaleMatrix2D(double scaleX, double scaleY);
	static CvMat* cutTo2x3Matrix(CvMat* matrix);
	static CvMat* generateTranslationMatrix2D(float translationX, float translationY);

	static void generateTranslationMatrix2D_(double translationX, double translationY,
	        CvMat* matrix);
	static CvMat* generateTranslationMatrix2D_(double translationX,
	        double translationY);
	static void generateRotationMatrix2D_(float angle, CvPoint2D64f center,
	        CvMat* matrix);
	static CvMat* generateRotationMatrix2D_(float angle, CvPoint2D64f center);
	static void generateRotationMatrix2D_(float angle, CvMat* matrix);
	static CvMat* generateRotationMatrix2D_(float angle);
	static void generateScaleMatrix2D_(double scaleX, double scaleY, CvMat* matrix);
	static CvMat* generateScaleMatrix2D_(double scaleX, double scaleY);
	static void copyMatrix33To23(CvMat* matrix33, CvMat* matrix23);
	static void PrintMatrix(const CvMat* matrix);
};

#endif /* CVMATHHELPER_H_ */
