/*
 * CvMathHelper.cpp
 *
 *  Created on: 27.09.2010
 *      Author: dube
 */

#include <iostream>
#include "CvMathHelper.h"

using namespace cv;
using namespace std;

CvMathHelper::CvMathHelper() {
	// TODO Auto-generated constructor stub

}

CvMathHelper::~CvMathHelper() {
	// TODO Auto-generated destructor stub
}


/*
 * Calculates the rotation matrix around z, y and x axis (in this order).
 */
Mat CvMathHelper::CreateRotationMatrix(double angleX, double angleY, double angleZ) {
	double cosx = cos(angleX);
	double cosy = cos(angleY);
	double cosz = cos(angleZ);
	double sinx = sin(angleX);
	double siny = sin(angleY);
	double sinz = sin(angleZ);
	Mat result = (Mat_<double>(4, 4) <<
			cosy * cosz, -cosy * sinz, siny, 0,
			cosx * sinz + sinx * siny * cosz, cosx * cosz - sinx *siny * sinz
			, -sinx * cosy, 0,
			sinx * sinz - cosx * siny * cosz,
			cosx * siny * sinz + sinx * cosz, cosx * cosy , 0,
			0, 0, 0, 1);
	return result;
}

/*
 * Calculates a translation matrix.
 */
Mat CvMathHelper::CreateTranslationMatrix(double deltaX, double deltaY, double deltaZ) {
	Mat result = (Mat_<double>(4, 4) <<
			1, 0, 0, deltaX,
			0, 1, 0, deltaY,
			0, 0, 1, deltaZ,
			0, 0, 0, 1);
	return result;
}

void CvMathHelper::PrintMatrix(const cv::Mat& matrix) {
    for (int row = 0; row < matrix.rows; row++) {
        for (int column = 0; column < matrix.cols; column++) {
            if (column > 0)
                cout << ", ";
            cout << matrix.at<double>(row, column);
        }
        cout << endl;
    }
}

void CvMathHelper::PrintMatrix(const CvMat* matrix) {
    for (int row = 0; row < matrix->rows; row++) {
        for (int column = 0; column < matrix->cols; column++) {
            if (column > 0)
                cout << ", ";
            cout << cvmGet(matrix, row, column);
        }
        cout << endl;
    }
}

Mat CvMathHelper::CreateVector(double x, double y, double z) {
	Mat result = (Mat_<double>(4, 1) << x, y, z, 1);
	return result;
}

CvMat* CvMathHelper::generateTranslationMatrix2D(float translationX, float translationY)
{
	//first we define an array with the matrix values
	float values[] =
	{ 1, 0, translationX, 0, 1, translationY, 0, 0, 1 };
	//next we generate a 3x3 matrix
	CvMat* translationMatrix = cvCreateMat(3, 3, CV_64FC1);
	//now we copy the array to the generated matrix
	//NOTE: Kind of hack to keep the code short... We also could assign the values one by one without the array.
	double* matrixValues = translationMatrix->data.db;
	for (int i = 0; i < 9; i++)
		matrixValues[i] = values[i];

	return translationMatrix;
}

void CvMathHelper::generateTranslationMatrix2D_(double translationX,
        double translationY,
        CvMat* matrix) {
    double* data = matrix->data.db;
    *data = 1; data++;
    *data = 0; data++;
    *data = translationX; data++;
    *data = 0; data++;
    *data = 1; data++;
    *data = translationY; data++;
    *data = 0; data++;
    *data = 0; data++;
    *data = 1; data++;
    //cout << "Translation matrix: " << endl;
    //PrintMatrix(matrix);
}

CvMat* CvMathHelper::generateTranslationMatrix2D_(double translationX,
        double translationY){
    CvMat* result= cvCreateMat(3, 3, CV_64FC1);
    generateTranslationMatrix2D_(translationX, translationY, result);
    return result;
}

void CvMathHelper::generateRotationMatrix2D_(float angle, CvPoint2D64f center,
        CvMat* matrix)
{
    //first we define an array with the matrix values
    float cosAngle = cos(angle);
    float sinAngle = sin(angle);
    double* data = matrix->data.db;

    *data = cosAngle; data++;
    *data = sinAngle; data++;
    *data = -(center.y) * sinAngle - (center.x) * cosAngle + center.x; data++;
    *data = -sinAngle; data++;
    *data = cosAngle; data++;
    *data = (center.x) * sinAngle - (center.y) * cosAngle + center.y; data++;
    *data = 0; data++;
    *data = 0; data++;
    *data = 1; data++;
    //cout << "Rotation matrix + center: " << endl;
    //PrintMatrix(matrix);
}

CvMat* CvMathHelper::generateRotationMatrix2D_(float angle, CvPoint2D64f center) {
    CvMat* result= cvCreateMat(3, 3, CV_64FC1);
    generateRotationMatrix2D_(angle, center);
    return result;
}


CvMat* CvMathHelper::generateRotationMatrix2D(float angle, CvPoint2D64f center)
{
    //first we define an array with the matrix values
    float cosAngle = cos(angle);
    float sinAngle = sin(angle);

    double values[] =
    { cosAngle, sinAngle, -(center.y) * sinAngle - (center.x) * cosAngle
            + center.x, -sinAngle, cosAngle, (center.x) * sinAngle - (center.y)
            * cosAngle + center.y, 0, 0, 1 };
    //next we generate a 3x3 matrix
    CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);
    //now we copy the array to the generated matrix
    //NOTE: Kind of hack to keep the code short... We also could assign the values one by one without the array.
    double* matrixValues = rotationMatrix->data.db;
    for (int i = 0; i < 9; i++)
        matrixValues[i] = values[i];

    return rotationMatrix;
}

void CvMathHelper::generateRotationMatrix2D_(float angle, CvMat* matrix)
{
    //first we define an array with the matrix values
    float cosAngle = cos(angle);
    float sinAngle = sin(angle);
    double* data = matrix->data.db;
    *data = cosAngle; data++;
    *data = sinAngle; data++;
    *data = 0; data++;
    *data = -sinAngle; data++;
    *data = cosAngle; data++;
    *data = 0; data++;
    *data = 0; data++;
    *data = 0; data++;
    *data = 1; data++;
    //cout << "Rotation matrix: " << endl;
    //PrintMatrix(matrix);
}

CvMat* CvMathHelper::generateRotationMatrix2D_(float angle) {
    CvMat* result= cvCreateMat(3, 3, CV_64FC1);
    generateRotationMatrix2D_(angle);
    return result;
}

CvMat* CvMathHelper::generateRotationMatrix2D(float angle)
{
	//first we define an array with the matrix values
	float cosAngle = cos(angle);
	float sinAngle = sin(angle);
	double values[] =
		{ cosAngle, sinAngle, 0, -sinAngle, cosAngle, 0, 0, 0, 1 };
	//next we generate a 3x3 matrix
	CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);
	//now we copy the array to the generated matrix
	//NOTE: Kind of hack to keep the code short... We also could assign the values one by one without the array.
	double* matrixValues = rotationMatrix->data.db;
	for (int i = 0; i < 9; i++)
		matrixValues[i] = values[i];

	return rotationMatrix;
}


void CvMathHelper::generateScaleMatrix2D_(double scaleX, double scaleY,
        CvMat* matrix)
{
    //first we define an array with the matrix values
    double* data = matrix->data.db;
    *data = scaleX; data++;
    *data = 0; data++;
    *data = 0; data++;
    *data = 0; data++;
    *data = scaleY; data++;
    *data = 0; data++;
    *data = 0; data++;
    *data = 0; data++;
    *data = 1; data++;
    //cout << "Scale matrix: " << endl;
    //PrintMatrix(matrix);
}

CvMat* CvMathHelper::generateScaleMatrix2D_(double scaleX, double scaleY) {
    CvMat* result= cvCreateMat(3, 3, CV_64FC1);
    generateScaleMatrix2D_(scaleX, scaleY);
    return result;
}

CvMat* CvMathHelper::generateScaleMatrix2D(double scaleX, double scaleY)
{
	//first we define an array with the matrix values
	double values[] =
	{ scaleX, 0, 0, 0, scaleY, 0, 0, 0, 1 };
	//next we generate a 3x3 matrix
	CvMat* scaleMatrix = cvCreateMat(3, 3, CV_64FC1);
	//now we copy the array to the generated matrix
	//NOTE: Kind of hack to keep the code short... We also could assign the values one by one without the array.
	double* matrixValues = scaleMatrix->data.db;
	for (int i = 0; i < 9; i++)
		matrixValues[i] = values[i];

	return scaleMatrix;
}

void CvMathHelper::copyMatrix33To23(CvMat* matrix33, CvMat* matrix23)
{
    double* data33 = matrix33->data.db;
    double* data23 = matrix23->data.db;
    data23[0] = data33[0];
    data23[1] = data33[1];
    data23[2] = data33[2];
    data23[3] = data33[3];
    data23[4] = data33[4];
    data23[5] = data33[5];
    //cout << "Copy matrix 33 to 32:" << endl;
    //PrintMatrix(matrix33);
    //PrintMatrix(matrix23);
}


CvMat* CvMathHelper::cutTo2x3Matrix(CvMat* matrix)
{
	CvMat* result = cvCreateMat(2, 3, CV_64FC1);
	for (int y = 0; y < 2; y++)
		for (int x = 0; x < 3; x++)
			cvmSet(result, y, x, cvmGet(matrix, y, x));
	return result;
}

void CvMathHelper::printCvPoint2D32f(CvPoint2D32f point)
{
	cout << "(" << point.x << ", " << point.y << ")" << endl;
}

