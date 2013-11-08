/*
 * CvContourManipulator.cpp
 *
 *  Created on: 25.11.2010
 *      Author: dube
 */

#include "RamaxxException.h"
#include "CvContourManipulator.h"


CvContourManipulator::CvContourManipulator()
{
	// TODO Auto-generated constructor stub

}

CvContourManipulator::~CvContourManipulator()
{
	// TODO Auto-generated destructor stub
}


CvSeq* CvContourManipulator::copyCvSeq(CvSeq* sequence)
{

	CvSeqReader reader;
	cvStartReadSeq(sequence, &reader, 0);

	CvSeqWriter writer;
	CvSeq* result = cvCreateSeq(sequence->flags, sequence->header_size,
			sequence->elem_size, sequence->storage);
	result->flags = sequence->flags;
	cvStartAppendToSeq(result, &writer);

	for (int i = 0; i < sequence->total; i++)
	{
		CvPoint* actualPoint = (CvPoint*) reader.ptr;
		CvPoint copiedPoint = cvPoint(actualPoint->x, actualPoint->y);
		CV_WRITE_SEQ_ELEM( copiedPoint, writer );
		CV_NEXT_SEQ_ELEM( sequence->elem_size, reader );
	}
	cvEndWriteSeq(&writer);

	CvRect boundingBox = ((CvContour*) sequence)->rect;
	((CvContour*) result)->rect = cvRect(boundingBox.x, boundingBox.y,
			boundingBox.width, boundingBox.height);

	return result;
}

CvSeq* CvContourManipulator::copyCvContour(CvSeq* contour, CvSeq* parent /*= NULL*/)
{
	CvSeq* sequence = contour;
	CvSeq* result = NULL;
	//At first we have to find the most left sequence
	while (sequence->h_prev != NULL)
		sequence = sequence->h_prev;
	//Now we go from left to right and copy the sequences
	CvSeq* previous = NULL;
	while (sequence != NULL)
	{
		CvSeq* newSequence = copyCvSeq(sequence);
		newSequence->v_prev = parent;
		//set links to the neighbors
		if (previous != NULL)
		{
			previous->h_next = newSequence;
			newSequence->h_prev = previous;
		}
		//copy child nodes
		if (sequence->v_next != NULL)
		{
			//cout << "child" << endl;
			CvSeq* child = copyCvContour(sequence->v_next, newSequence);
			newSequence->v_next = child;
		}
		//copy parent nodes
		//FIXME: We do not copy parent nodes because it would increase the risk of running in loops.
		//	For the simple blob sequences it should work perfectly anyway.

		//Take care that we return the correct sequence.
		if (sequence == contour)
			result = newSequence;

		previous = newSequence;
		sequence = sequence->h_next;
	}
	assert(result != NULL);
	return result;
}

CvSeq* CvContourManipulator::transformContour(CvSeq* contourSequence,
		CvMat* transformationMatrix)
{
	//CvSeq *tmpPoints;
	CvSeqReader reader;
	CvSeqWriter writer;
	CvRect boundingBox;
	CvSeq* transformedContour;

	boundingBox = ((CvContour*) contourSequence) ->rect;
	//cout << "Bounding box: " << boundingBox.x << " " << boundingBox.y << " " << boundingBox.width << " " << boundingBox.height << endl;
	// apply an offset to contour points to recover real coordinates

	cvStartReadSeq(contourSequence, &reader);

	transformedContour = cvCreateSeq(contourSequence->flags,
			contourSequence->header_size, contourSequence->elem_size,
			contourSequence->storage);
	cvStartAppendToSeq(transformedContour, &writer);

	// also calculate bounding box of the contour to allow cvPointPolygonTest
	// work correctly on the generated polygon
	boundingBox.x = boundingBox.y = INT_MAX;
	boundingBox.width = boundingBox.height = INT_MIN;

	//cout << "Transformation Matrix: " << endl;
	//printMatrix(transformationMatrix);
	double* matrix = transformationMatrix->data.db;

	for (int i = 0; i < contourSequence->total; i++)
	{
		CvPoint actualPoint;
		CV_READ_SEQ_ELEM( actualPoint, reader);

		//cout << "old Point: (" << actualPoint.x << "x" << actualPoint.y << "); ";

		CvPoint newPoint = cvPoint(floor(actualPoint.x * matrix[0]
				+ actualPoint.y * matrix[1] + matrix[2] + 0.5), floor(
				actualPoint.x * matrix[3] + actualPoint.y * matrix[4]
						+ matrix[5] + 0.5));

		//cout << "new Point: (" << newPoint.x << "x" << newPoint.y << "); " << endl,

		boundingBox.x = MIN( boundingBox.x, newPoint.x );
		boundingBox.y = MIN( boundingBox.y, newPoint.y );
		boundingBox.width = MAX( boundingBox.width, newPoint.x );
		boundingBox.height = MAX( boundingBox.height, newPoint.y );

		CV_WRITE_SEQ_ELEM( newPoint, writer );
	}
	cvEndWriteSeq(&writer);
	//cvClearSeq( contourSequence );

	boundingBox.width = boundingBox.width - boundingBox.x;
	boundingBox.height = boundingBox.height - boundingBox.y;

	// assign calculated bounding box
	((CvContour*) transformedContour)->rect = boundingBox;

	return transformedContour;
}

void CvContourManipulator::transformContourInPlace(CvSeq* contourSequence,
		CvMat* transformationMatrix)
{
	if (contourSequence == NULL)
		throw RamaxxException("Contour is empty.");

	CvRect boundingBox;
	boundingBox = ((CvContour*) contourSequence)->rect;

	// also calculate bounding box of the contour to allow cvPointPolygonTest
	// work correctly on the generated polygon
	boundingBox.x = boundingBox.y = INT_MAX;
	boundingBox.width = boundingBox.height = INT_MIN;
	//Init some variables
	double* matrix = transformationMatrix->data.db;
	int tempInt;
	CvSeqReader reader;
	cvStartReadSeq(contourSequence, &reader, 0);

	for (int i = 0; i < contourSequence->total; i++)
	{
		CvPoint* actualPoint = (CvPoint*) reader.ptr;
		//tempInt is the temporal storage for the x value so we can override it directly in the point.
		tempInt = actualPoint->x;
		//cout << "old Point: (" << actualPoint->x << "x" << actualPoint->y << "); ";
		actualPoint->x = floor(actualPoint->x * matrix[0] + actualPoint->y
				* matrix[1] + matrix[2] + 0.5);
		actualPoint->y = floor(tempInt * matrix[3] + actualPoint->y * matrix[4]
				+ matrix[5] + 0.5);
		//cout << "new Point: (" << actualPoint->x << "x" << actualPoint->y << "); " << endl;

		boundingBox.x = MIN( boundingBox.x, actualPoint->x );
		boundingBox.y = MIN( boundingBox.y, actualPoint->y );
		boundingBox.width = MAX( boundingBox.width, actualPoint->x );
		boundingBox.height = MAX( boundingBox.height, actualPoint->y );
		CV_NEXT_SEQ_ELEM( contourSequence->elem_size, reader );
	}

	//Calculate width and height
	boundingBox.width = boundingBox.width - boundingBox.x;
	boundingBox.height = boundingBox.height - boundingBox.y;

	// assign calculated bounding box
	((CvContour*) contourSequence)->rect = boundingBox;
}

void CvContourManipulator::transformContourTree(CvSeq* contour,
		CvMat* transformationMatrix)
{
	//Transform this contour
	transformContourInPlace(contour, transformationMatrix);

	//Transform recursively all neighbor and child contours
	if (contour->h_next != NULL)
		transformContourTree(contour->h_next, transformationMatrix);
	if (contour->v_next != NULL)
		transformContourTree(contour->v_next, transformationMatrix);
}

void CvContourManipulator::drawContour(IplImage* image, CvSeq* contour,
		CvScalar foregroundColor, CvScalar backgroundColor)
{
	CvRect boundingBox = ((CvContour*) contour)->rect;
	//	if (image->width != (boundingBox.width + boundingBox.x) || image->height != (boundingBox.height + boundingBox.y)) {
	//		cout << "WARNING: Size of destination image (" << image->width << "x" << image->height << ") ";
	//		cout << "does not match the blob size (" << boundingBox.width << "x" << boundingBox.height << ")." << endl;
	//	}
	fillImage(image, backgroundColor.val[0]);
	cvDrawContours(image, contour, foregroundColor, backgroundColor, 2,
			CV_FILLED, 8);
}

void CvContourManipulator::fillImage(IplImage* image, int value)
{
	if (image->nChannels != 1)
	{
		throw RamaxxException("Can not fill image with more than one channel.");
		return;
	}
	if (image->depth != IPL_DEPTH_8U)
	{
		throw RamaxxException("Can only fill image with 8 bit color depth.");
		return;
	}

	for (int y = 0; y < image->height; y++)
		for (int x = 0; x < image->width; x++)
		{
			((uchar *) (image->imageData + y * image->widthStep))[x] = value;
		}
}
