/*
 * CvContourManipulator.h
 *
 *  Created on: 25.11.2010
 *      Author: dube
 */

#ifndef CVCONTOURMANIPULATOR_H_
#define CVCONTOURMANIPULATOR_H_

#include <stddef.h>
#include <opencv/cv.h>
#include <string>
#include <exception>

class CvContourManipulator
{
public:
	CvContourManipulator();
	virtual ~CvContourManipulator();

	static void transformContourTree(CvSeq* contour, CvMat* transformationMatrix);
	static void drawContour(IplImage* image, CvSeq* contour, CvScalar foregroundColor, CvScalar backgroundColor );
	static CvSeq* copyCvSeq(CvSeq* sequence);
	static CvSeq* copyCvContour(CvSeq* contour, CvSeq* parent = NULL);
	static CvSeq* transformContour(CvSeq* contourSequence, CvMat* transformationMatrix);
	static void transformContourInPlace(CvSeq* contourSequence, CvMat* transformationMatrix);
	static void fillImage(IplImage* image, int value);
};

#endif /* CVCONTOURMANIPULATOR_H_ */
