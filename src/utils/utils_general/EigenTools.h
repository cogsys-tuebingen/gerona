/*
 * EigenTools.h
 *
 *  Created on: Aug 11, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#ifndef EIGENTOOLS_H_
#define EIGENTOOLS_H_

#include <Eigen/Core>

// Returns the matrix corresonding to a rotation around in 2d around the angle phi
Eigen::Matrix2d RotMat2d(double phi);

// Returns the matrix corresonding to a rotation around the x axis by the angle phi
Eigen::Matrix3d RotMatX(double phi);

// Returns the matrix corresonding to a rotation around the y axis by the angle phi
Eigen::Matrix3d RotMatY(double phi);

// Returns the matrix corresonding to a rotation around the z axis by the angle phi
Eigen::Matrix3d RotMatZ(double phi);

// Returns the rotation matrix corresonding to given roll pitch yaw angles.
Eigen::Matrix3d RPYToMatrix(Eigen::Vector3d rpy);

#endif /* EIGENTOOLS_H_ */
